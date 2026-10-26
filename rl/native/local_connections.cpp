// Local geometry only: no ROS, map search, or complete-path planner.
#include <fwis_common/reeds_shepp.hpp>
#include <algorithm>

extern "C" int coptrans_connections(const double* p, double* out, int capacity) {
  const double radius=p[6];
  if (!(radius>0) || capacity<1) return 0;
  int count=0;
  auto append=[&](const std::vector<fwis::RSSeg>& segments) {
    if (count>=capacity || segments.size()>5) return;
    double* row=out+16*count;
    std::fill(row,row+16,0.0);
    int n=0;
    for (const auto& s:segments) {
      if(s.length<1e-9) continue;
      row[1+2*n]=s.direction*s.length;
      row[2+2*n]=(s.type=='L'?1.0:s.type=='R'?-1.0:0.0)/radius;
      ++n;
    }
    row[0]=n;
    if(n) ++count;
  };
  append(fwis::reeds_shepp(p[0],p[1],p[2],p[3],p[4],p[5],radius).segs);
  // Four existing CSC formula families, also with all-backward travel.
  // These are bounded proposals, not an exhaustive Dubins/RS implementation.
  for(int gear:{1,-1}) {
    double x,y,phi;
    fwis::rs_impl::to_local(p[0],p[1],p[2]+(gear<0?M_PI:0),
                          p[3],p[4],p[5]+(gear<0?M_PI:0),radius,x,y,phi);
    for(int reflection:{1,-1}) for(int opposite:{0,1}) {
      const auto word=opposite?fwis::rs_impl::LpSpRp(x,reflection*y,reflection*phi):
                                fwis::rs_impl::LpSpLp(x,reflection*y,reflection*phi);
      if(!word.ok) continue;
      const char first=reflection*gear>0?'L':'R';
      const char last=opposite?(first=='L'?'R':'L'):first;
      append({{first,word.t*radius,gear},{'S',word.u*radius,gear},
              {last,word.v*radius,gear}});
    }
  }
  return count;
}
