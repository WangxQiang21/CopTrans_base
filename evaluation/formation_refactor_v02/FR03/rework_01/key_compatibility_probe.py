"""Codex FR03 local successor/key compatibility probe; no global search."""
import dataclasses
import json
from pathlib import Path
import sys
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'tests').is_dir() and (p/'catkin_ws').is_dir())
sys.path.insert(0,str(ROOT/'tests'))
import test_uniform_search as f
OUT=Path(__file__).resolve().parent
cfg=f.config();gen=f.generator(cfg=cfg);qf=f.FormationPose(0.,0.,0.);goal=f.FormationPose(1.,0.,0.)
rows=[]
for sign in (1.,-1.):
    prefix=f.s_spin_edge(f.model(),qf,f.headings(sign*.02),f.headings(-sign*.01),limits=gen.motion_limits)
    parent=f.state_from_edge(prefix)
    before=gen.generate(parent,goal)
    assert not any(x.edge.primitive=='A_STRAIGHT' for x in before.successors)
    align=next(x for x in before.successors if x.edge.primitive=='S_SPIN' and all(abs(h)<1e-12 for _,h in x.edge.end_headings))
    after=gen.generate(align.next_state,goal)
    straight=next(x for x in after.successors if x.edge.primitive=='A_STRAIGHT' and x.label=='A_straight_goal_gear+++')
    audited=f.audit_edges((prefix,align.edge,straight.edge));assert audited['feasible']
    table=f.RepresentativeTable(cfg);first=table.offer(parent)[0];second=table.offer(align.next_state)[0]
    assert first
    same=f.make_key(parent,cfg)==f.make_key(align.next_state,cfg)
    rows.append({'sign':sign,'parent':parent.as_dict(),'aligned_child':align.next_state.as_dict(),'parent_key':f.make_key(parent,cfg).as_dict(),'child_key':f.make_key(align.next_state,cfg).as_dict(),'same_key':same,'table_kept_aligned_child':second,'parent_direct_A_count':0,'child_direct_A_label':straight.label,'alignment_cost_s':align.cost_s,'full_prefix_alignment_straight_FR02':audited,'status':'FAILED' if same and not second else 'PASSED'})
(OUT/'key_compatibility_probe.json').write_text(json.dumps({'scope':'FR03 local states/generator/representative only','cases':rows},ensure_ascii=False,indent=2)+'\n')
print(json.dumps([{'sign':r['sign'],'same_key':r['same_key'],'kept_child':r['table_kept_aligned_child'],'cost':r['alignment_cost_s'],'FR02':r['full_prefix_alignment_straight_FR02']['status'],'status':r['status']} for r in rows],indent=2))
sys.exit(1 if any(r['status']=='FAILED' for r in rows) else 0)
