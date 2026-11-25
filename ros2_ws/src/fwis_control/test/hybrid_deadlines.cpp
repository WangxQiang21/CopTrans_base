// Native soft-real-time integration bench. No DDS or simulation-time clock here.
#include "fwis_control/tracking_core.hpp"
#include "fwis_hybrid_joint/model.hpp"
#include "../src/control_affinity.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace fc=fwis_control;
namespace hj=fwis_hybrid_joint;
using namespace std::chrono;
using Clock=steady_clock;
struct Tick { long release, jitter, compute, submitted; bool valid; };
long ns(Clock::duration d) { return duration_cast<nanoseconds>(d).count(); }
int main(int argc,char** argv) {
  if(argc<4) { std::cerr<<"hybrid_deadlines VEHICLES SECONDS CSV [WORKER_CPU ...]\n"; return 2; }
  const int cars=std::stoi(argv[1]), seconds=std::stoi(argv[2]);
  if((cars!=1 && cars!=3) || seconds<1 || seconds>3600) return 2;
  if(argc!=4 && argc!=4+cars) return 2;
  std::vector<int> worker_cpus(cars,-1);
  for(int i=4;i<argc;++i) {
    const int cpu=std::stoi(argv[i]);
    if(cpu<0 || cpu>=CPU_SETSIZE) return 2;
    worker_cpus[i-4]=cpu;
  }
  const int ticks=seconds*25+1;
  hj::Request r;
  r.grid=std::make_shared<hj::GridSnapshot>(1200,120,.10,fwis_planning_core::Point2d{-5.,-5.},std::vector<uint8_t>(144000,0));
  r.vehicles=cars==1?std::vector<hj::Footprint>{{0,0,.818,.58}}:
    std::vector<hj::Footprint>{{1.,0,.838,.6},{0,1.,.838,.6},{0,-1.,.838,.6}};
  if(cars==3)r.cargo=hj::Footprint{.5,0,2.05,2.8};
  r.goal.reference={30.,0.,0.};
  hj::Piece piece;
  for(int i=0;i<6;++i)piece.x[i]=6.*i;
  hj::Phase phase;
  phase.duration=std::max(300.,seconds+60.);phase.pieces.push_back(piece);
  r.task_duration_cap=phase.duration+1.;
  hj::Plan authority{{phase}};
  const auto audit=hj::certify(r,authority,Clock::now()+std::chrono::seconds(5));
  if(!audit.executable){std::cerr<<audit.reason<<"\n";return 3;}
  fc::Configuration config;
  std::vector<fc::PreparedPlan> plans;
  for(int v=0;v<cars;++v) {
    const auto sample=[&,v](double t){
      auto b=hj::evaluate(r,authority,t).bodies[v];
      return fc::Reference{t,{b.pose.x,b.pose.y,b.pose.yaw},{b.vx,b.vy,b.wz},fc::Mode::Ackermann};
    };
    fc::Trajectory trajectory{1000000000,"map",1,{}};
    for(int j=0;j<=static_cast<int>(phase.duration)*10;++j){
      const double t=j*.1;auto ref=sample(t);
      trajectory.points.push_back({j*100000000LL,ref.pose,ref.twist.vx,0});
    }
    plans.push_back(fc::prepare(std::move(trajectory),config,false));
    if(!plans.back().matching_mode.valid) return 4;
    plans.back().model_sample=sample;
  }
  std::atomic<bool> running{true}; std::atomic<int> completed{},certified{};
  std::thread load([&]{
    auto task=r;task.goal.reference={4.,.4,.1};task.goal.heading.fill(.17);
    if(cars==1)task.goal.heading[0]=task.goal.reference.yaw;
    while(running.load()) {
      const auto result=hj::solve(task,Clock::now()+std::chrono::seconds(cars==1?8:12),
                                   [&]{return !running.load();});
      if(result.plan) ++certified;
      if(running.load()) ++completed;
    }
  });
  std::vector<std::thread> workers;
  std::array<std::atomic<double>,9> command_bus{};
  std::vector<std::vector<Tick>> samples(cars,std::vector<Tick>(ticks));
  const auto epoch=Clock::now()+milliseconds(250);
  for(int v=0;v<cars;++v) workers.emplace_back([&,v]{
    fc::bind_control_cpu(worker_cpus[v]);
    fc::Execution tracking(config,false);
    tracking.admit(plans[v],fc::Mode::Ackermann,1.);
    for(int k=0;k<ticks;++k){
      const auto expected=epoch+milliseconds(40LL*k);
      std::this_thread::sleep_until(expected);
      const auto begin=Clock::now();const auto elapsed=duration<double>(begin-epoch).count();
      auto ref=plans[v].model_sample(elapsed);
      const double now=1.+elapsed;
      fc::Observation feedback{ref.pose,ref.twist,now,fc::Mode::Ackermann,now};
      const auto before=Clock::now();
      auto cmd=tracking.tick(now,feedback);
      const auto after=Clock::now();
      command_bus[3*v].store(cmd.command.vx);command_bus[3*v+1].store(cmd.command.vy);
      command_bus[3*v+2].store(cmd.command.wz);
      const auto submitted=Clock::now();
      samples[v][k]={ns(expected-epoch),ns(begin-expected),ns(after-before),ns(submitted-epoch),
                     cmd.phase==fc::Phase::Tracking && cmd.command.finite()};
    }
  });
  for(auto& w:workers)w.join();
  running=false;load.join();
  std::ofstream file(argv[3]); if(!file)return 5;
  file<<"vehicle,tick,scheduled_ns,release_jitter_ns,tracking_compute_ns,command_submitted_ns,valid\n";
  for(int v=0;v<cars;++v)for(int k=0;k<ticks;++k) {
    const auto& s=samples[v][k];
    file<<v+1<<','<<k<<','<<s.release<<','<<s.jitter<<','<<s.compute<<','<<s.submitted<<','<<s.valid<<'\n';
  }
  std::cout<<"{\"worker_cpus\":[";
  for(int v=0;v<cars;++v) std::cout<<(v?",":"")<<worker_cpus[v];
  std::cout<<"],\"completed_planner_requests\":"<<completed<<",\"certified_planner_requests\":"<<certified
           <<",\"wall_s\":"<<duration<double>(Clock::now()-epoch).count()<<"}\n";
  return completed>0 && certified>0 && file.good()?0:6;
}
