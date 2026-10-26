"""Independent bounded frontend: select mode+geometry edges in a live frontier."""
from dataclasses import dataclass
import math
import numbers
import time

import numpy as np
from scipy.ndimage import zoom

from .kernel_adapter import wrap, advance, finalize_candidate, FinalizationConfig
from .rule_candidates import RuleCandidates


@dataclass(frozen=True)
class FrontendConfig:
    capacity: int = 64
    max_decisions: int = 128
    max_finalizations: int = 4
    budget_s: float = 10.
    finalize_reserve_s: float = 1.
    max_execution_s: float = 300.

    def __post_init__(self):
        for name in ("capacity","max_decisions","max_finalizations"):
            value = getattr(self,name)
            if isinstance(value,bool) or not isinstance(value,numbers.Integral) or value<1:
                raise ValueError("invalid_frontend_config:"+name)
        if (not math.isfinite(self.budget_s) or not math.isfinite(self.finalize_reserve_s) or
                not 0 < self.finalize_reserve_s < self.budget_s):
            raise ValueError("invalid_frontend_deadline")
        if not math.isfinite(self.max_execution_s) or self.max_execution_s<=0:
            raise ValueError("invalid_execution_budget")


@dataclass(frozen=True)
class Branch:
    parent_pose: object
    actions: tuple
    edge: object
    proxy: float
    depth: int


class RuleFrontend:
    schema = "rule_frontier_mode_path_v1"
    feature_dim = 27

    def __init__(self, config=None):
        self.config = config or FrontendConfig()
        self.done = True

    def at_goal(self, pose):
        p = self.problem
        return (math.hypot(pose.x-p.goal.x,pose.y-p.goal.y)<=p.xy_tol and
                abs(wrap(pose.th-p.goal.th))<=p.yaw_tol)

    def heuristic(self, pose):
        p = self.problem
        distance = self.generator.guide.distance_to_goal(pose,p.goal)/p.limits.target_v
        angle = abs(wrap(p.goal.th-pose.th))/max(p.limits.omega_max,p.limits.translation_omega_max or 0.)
        return max(distance,angle)

    def priority(self, branch):
        # Search preference only; final selection always uses certified time.
        return branch.proxy+1.5*self.heuristic(branch.edge.end)-2*int(self.at_goal(branch.edge.end))

    def state_key(self, branch):
        end = branch.edge.end
        gear, reversals, spin = 0,0,0.
        for a in branch.actions:
            if a.mode == 2:
                spin += a.amount
            else:
                new = 1 if a.amount>0 else -1
                reversals += int(gear!=0 and new!=gear)
                gear,spin = new,0.
        last = branch.actions[-1]
        return (round(end.x/.05),round(end.y/.05),round(end.th/math.radians(3)),
                last.mode,round(last.control,4),gear,reversals,round(wrap(spin),4))

    def _add(self, pose, prefix, proxy, depth):
        stop = self.deadline-self.config.finalize_reserve_s
        if time.perf_counter() >= stop:
            return
        for edge in self.generator.generate(pose,prefix,stop):
            actions = (*prefix,*edge.actions)
            branch = Branch(pose,actions,edge,proxy+edge.proxy_s,depth+1)
            if branch.proxy >= self.expanded.get(self.state_key(branch),float("inf"))-1e-8:
                continue
            self.frontier.append(branch)
        # Reserve representatives of different endpoint sectors and last modes.
        # This cap is approximate pruning, not a completeness guarantee.
        self.frontier.sort(key=self.priority)
        groups, selected, rest = set(),[],[]
        for b in self.frontier:
            angle = math.atan2(b.edge.end.y-self.problem.start.y,b.edge.end.x-self.problem.start.x)
            group = (b.actions[-1].mode,round(angle/(math.pi/4)),self.at_goal(b.edge.end))
            if group not in groups:
                selected.append(b); groups.add(group)
            else:
                rest.append(b)
        self.frontier = sorted((selected+rest)[:self.config.capacity],key=self.priority)

    def reset(self, problem):
        self.begun = time.perf_counter()
        self.deadline = self.begun+self.config.budget_s
        self.problem = problem
        if problem.initial_mode != 0:
            raise ValueError("unsupported_initial_mode")
        if not all(problem.grid.pose_clear(p) for p in (problem.start,problem.goal)):
            raise ValueError("invalid_start_or_goal_footprint")
        self.generator = RuleCandidates(problem)
        self.frontier, self.expanded, self.events = [],{},[]
        self.steps = self.finalizations = 0
        self.best = None
        self.best_actions = ()
        self.last_actions = ()
        self.stop_reason = None
        self.done = False
        if self.at_goal(problem.start):
            self.best = finalize_candidate(problem,[],FinalizationConfig(include_metrics=False),self.deadline)
            self.done = True
            self.stop_reason = "initial_goal"
        else:
            self._add(problem.start,(),0.,0)
            self.done = not self.frontier or time.perf_counter()>=self.deadline
            if self.done:
                self.stop_reason = "deadline" if time.perf_counter()>=self.deadline else "no_initial_candidate"
        guide = self.generator.guide
        distance = np.where(np.isfinite(guide.distance),guide.distance,100.)
        shape = np.asarray(guide.occ.shape,dtype=float)
        # Fixed spatial coordinates, no labels/teachers in the observation.
        self.map_image = np.stack([zoom(guide.occ.astype(float),32/shape,order=0),
                                   zoom(np.minimum(distance/20.,1.),32/shape,order=1)]).astype(np.float32)
        return self.observe()

    def observe(self):
        p = self.problem
        features = np.zeros((self.config.capacity,self.feature_dim),dtype=np.float32)
        logits = np.zeros(self.config.capacity,dtype=np.float32)
        mask = np.zeros(self.config.capacity,dtype=bool)
        for i,b in enumerate(self.frontier):
            end = b.edge.end
            dx,dy = p.goal.x-end.x,p.goal.y-end.y
            c,s = math.cos(end.th),math.sin(end.th)
            length = sum(abs(a.amount) for a in b.edge.actions if a.mode!=2)
            spin = sum(abs(a.amount) for a in b.edge.actions if a.mode==2)
            switches = sum(a.mode!=z.mode for a,z in zip(b.actions,b.actions[1:]))
            switches += int(b.actions[0].mode != p.initial_mode)
            reverse = sum(abs(a.amount) for a in b.actions if a.mode!=2 and a.amount<0)
            clearance = p.grid.center_clearance_lower_bound(end.x,end.y)
            features[i] = [(c*dx+s*dy)/p.length,(-s*dx+c*dy)/p.length,
                           math.sin(p.goal.th-end.th),math.cos(p.goal.th-end.th),
                           (end.x-p.grid.ox)/(p.grid.width*p.grid.res),
                           (end.y-p.grid.oy)/(p.grid.height*p.grid.res),
                           *[float(b.actions[-1].mode==m) for m in range(3)],
                           float(self.at_goal(end)),b.proxy/10.,self.heuristic(end)/10.,
                           b.edge.proxy_s/10.,b.depth/self.config.max_decisions,
                           length/p.length,spin/math.pi,switches/10.,reverse/p.length,
                           min(clearance/p.length,2.),
                           (self.heuristic(b.parent_pose)-self.heuristic(end))/10.,
                           b.edge.actions[0].mode/2.,len(b.edge.actions)/5.,
                           math.hypot(dx,dy)/p.length,abs(wrap(p.goal.th-end.th))/math.pi,
                           self.best.timed.duration/self.config.max_execution_s if self.best else 1.,
                           1.-self.steps/self.config.max_decisions,
                           1.-self.finalizations/self.config.max_finalizations]
            logits[i] = -self.priority(b)
            mask[i] = True
        return {"candidates":np.clip(features,-100.,100.),"rule_logits":logits,
                "mask":mask,"map":self.map_image.copy()}

    def step(self, index):
        if self.done:
            raise RuntimeError("step_after_frontend_done")
        if (isinstance(index,bool) or not isinstance(index,numbers.Integral) or
                not 0<=index<len(self.frontier)):
            raise ValueError("invalid_frontier_choice")
        branch = self.frontier.pop(int(index))
        self.last_actions = branch.actions
        self.steps += 1
        reward = 0.
        event = {"decision":self.steps,"source":branch.edge.source,
                 "modes":[a.mode for a in branch.edge.actions],"depth":branch.depth}
        if self.at_goal(branch.edge.end):
            self.finalizations += 1
            result = finalize_candidate(self.problem,branch.actions,
                FinalizationConfig(include_metrics=False),self.deadline,source="rule_rl_frontend")
            event.update(status=result.status,reason=result.reason)
            if result.feasible and result.timed.duration>self.config.max_execution_s:
                event.update(status="REJECTED",reason="execution_time_budget_exceeded")
            if (result.feasible and result.timed.duration<=self.config.max_execution_s and
                    (self.best is None or result.timed.duration < self.best.timed.duration)):
                reward = (50.-result.timed.duration/10. if self.best is None else
                          (self.best.timed.duration-result.timed.duration)/10.)
                self.best,self.best_actions = result,branch.actions
                event["incumbent_execution_s"] = result.timed.duration
        else:
            key = self.state_key(branch)
            if branch.proxy < self.expanded.get(key,float("inf"))-1e-8:
                self.expanded[key] = branch.proxy
                self._add(branch.edge.end,branch.actions,branch.proxy,branch.depth)
        self.events.append(event)
        self.frontier = [b for b in self.frontier if self.at_goal(b.edge.end) or
                         b.proxy < self.expanded.get(self.state_key(b),float("inf"))-1e-8]
        self.done = (self.steps>=self.config.max_decisions or not self.frontier or
                     self.finalizations>=self.config.max_finalizations or
                     time.perf_counter()>=self.deadline-self.config.finalize_reserve_s)
        if self.done:
            self.stop_reason = ("deadline" if time.perf_counter()>=self.deadline-self.config.finalize_reserve_s else
                "finalization_limit" if self.finalizations>=self.config.max_finalizations else
                "decision_budget" if self.steps>=self.config.max_decisions else "frontier_exhausted")
        if self.done and self.best is None:
            reward -= 50.
        return self.observe(),reward,self.done

    def report(self, include_trace=True):
        success = self.best is not None and self.best.feasible
        return {"task_id":self.problem.task_id,"is_success":success,
                "execution_s":self.best.timed.duration if success else None,
                "wall_s":time.perf_counter()-self.begun,"decisions":self.steps,
                "finalizations":self.finalizations,"expanded":len(self.expanded),
                "reason":"certified" if success else self.stop_reason,
                "stop_reason":self.stop_reason,
                "last_attempt":[[a.mode,a.amount,a.control] for a in self.last_actions],
                "candidate_generation":dict(self.generator.stats),"events":list(self.events),
                "finalization":self.best.report(include_trace) if success else None}


def solve(problem, config=None, selector=None, include_trace=True):
    env = RuleFrontend(config)
    observation = env.reset(problem)
    while not env.done:
        index = (int(np.argmax(np.where(observation["mask"],observation["rule_logits"],-np.inf)))
                 if selector is None else selector(observation))
        observation,_,_ = env.step(index)
    return env.report(include_trace)
