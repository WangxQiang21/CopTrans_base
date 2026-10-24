// Compile the actual planner types and action-policy checker. No ROS master
// or PlannerNode instance is needed for these state/history contracts.
#include <gtest/gtest.h>
#include <random>
#define main planner_ros_entry_not_used_by_policy_tests
#include "../src/planner_node.cpp"
#undef main

TEST(PlannerPolicyHistory, DistinctHistoriesDoNotMergeAtTheSamePose) {
  Key fresh{12, 8, 0, A, 1, 0, 0, 1, 1, 1, 0, 0};
  Key spun = fresh;
  spun.spun_since_translation = 1;
  Key reversed = fresh;
  reversed.shunt_reversals = 1;
  EXPECT_FALSE(fresh == spun);
  EXPECT_FALSE(fresh == reversed);
  EXPECT_FALSE(spun == reversed);
  std::unordered_map<Key, int, KeyHash> histories;
  histories[fresh] = 10;
  histories[spun] = 20;
  histories[reversed] = 30;
  ASSERT_EQ(histories.size(), 3u);
  EXPECT_EQ(histories[fresh], 10);
  EXPECT_EQ(histories[spun], 20);
  EXPECT_EQ(histories[reversed], 30);
}

TEST(PlannerPolicyHistory, ActualSpinMakesOneReversalLegal) {
  EXPECT_EQ(shunting_violation({{A, 1., 0.}, {O, -.4, .2}}),
            "translation_reversal_without_spin");
  EXPECT_TRUE(shunting_violation({{A, 1., 0.}, {S, .4, 0.}, {O, -.4, .2}}).empty());
  EXPECT_TRUE(shunting_violation({{A, -1., .2}, {O, -.4, -.2}}).empty());
}

TEST(PlannerPolicyHistory, SpinDoesNotPermitASecondReversal) {
  EXPECT_EQ(shunting_violation({{A, 1., 0.}, {S, .4, 0.}, {O, -.4, .2},
                                {S, -.3, 0.}, {A, .2, 0.}}),
            "repeated_translation_reversals");
}

TEST(PlannerPolicyHistory, ZeroOrCancelledSpinDoesNotBypassPolicy) {
  EXPECT_EQ(shunting_violation({{A, 1., 0.}, {S, 0., 0.}, {O, -.4, .2}}),
            "translation_reversal_without_spin");
  EXPECT_EQ(shunting_violation({{A, 1., 0.}, {S, .4, 0.}, {S, -.4, 0.}, {A, -.2, 0.}}),
            "translation_reversal_without_spin");
}


TEST(PlannerObstacleSlabs, PreservesEveryOccupiedCellAndCompressesStraightWalls) {
  std::vector<std::vector<CellRun>> rows(200);
  for(int y=20;y<180;++y) rows[y]={{10,30},{100,120}};
  const auto slabs=merge_occupied_rows(rows);
  ASSERT_EQ(slabs.size(),3u);
  for(const auto& slab:slabs) for(int y=slab.y0;y<=slab.y1;++y) {
    EXPECT_EQ(slab.runs,rows[y]);
  }
}

TEST(PlannerObstacleSlabs, MatchesPerCellSATOnRotatedBodiesAndIrregularRaster) {
  const int w=60,h=55;
  const double res=.08,hl=.389,hw=.27;
  std::mt19937 rng(20261018);
  std::uniform_real_distribution<double> unit(0.,1.);
  for(int layout=0;layout<3;++layout) {
    std::vector<std::vector<CellRun>> rows(h);
    std::vector<std::pair<int,int>> cells;
    for(int y=0;y<h;++y) for(int x=0;x<w;++x) {
      const bool occupied=(layout==0? unit(rng)<.04 :
          layout==1? (y>=10&&y<45&&x>=20&&x<32) : true);
      if(!occupied) continue;
      cells.emplace_back(x,y);
      auto& row=rows[y];
      if(row.empty()||row.back().x1!=x-1) row.push_back({x,x});
      else row.back().x1=x;
    }
    const auto slabs=merge_occupied_rows(rows);
    for(int trial=0;trial<1500;++trial) {
      const Pose p{.5+3.8*unit(rng),.5+3.3*unit(rng),(2*unit(rng)-1)*PI};
      const double c=std::cos(p.th),s=std::sin(p.th),half=res/2;
      const double rx=hl*std::abs(c)+hw*std::abs(s),ry=hl*std::abs(s)+hw*std::abs(c);
      bool expected=true;
      for(const auto& cell:cells) {
        const double dx=(cell.first+.5)*res-p.x,dy=(cell.second+.5)*res-p.y;
        if(std::abs(dx*c+dy*s)<=hl+half*(std::abs(c)+std::abs(s)) &&
           std::abs(-dx*s+dy*c)<=hw+half*(std::abs(c)+std::abs(s)) &&
           std::abs(dx)<=half+rx && std::abs(dy)<=half+ry) {expected=false;break;}
      }
      EXPECT_EQ(occupied_slabs_clear(slabs,p,hl,hw,res,0.,0.,
          std::max(0,int(std::floor((p.x-rx)/res))-1),
          std::min(w-1,int(std::floor((p.x+rx)/res))+1),
          std::max(0,int(std::floor((p.y-ry)/res))-1),
          std::min(h-1,int(std::floor((p.y+ry)/res))+1)),expected);
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
