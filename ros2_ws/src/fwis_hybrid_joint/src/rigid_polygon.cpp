#include "rigid_polygon.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fwis_hybrid_joint {
namespace {
using fwis_planning_core::Point2d;
using Triangle = std::array<Point2d, 3>;

long double turn(Point2d a, Point2d b, Point2d c) {
  const long double ax = a.x, ay = a.y;
  return (b.x - ax) * (c.y - ay) - (b.y - ay) * (c.x - ax);
}

bool on_segment(Point2d a, Point2d b, Point2d p) {
  return turn(a, b, p) == 0 && std::min(a.x,b.x) <= p.x && p.x <= std::max(a.x,b.x) &&
    std::min(a.y,b.y) <= p.y && p.y <= std::max(a.y,b.y);
}

bool segments_intersect(Point2d a, Point2d b, Point2d c, Point2d d) {
  const auto ac=turn(a,b,c), ad=turn(a,b,d), ca=turn(c,d,a), cb=turn(c,d,b);
  if (on_segment(a,b,c) || on_segment(a,b,d) || on_segment(c,d,a) || on_segment(c,d,b))
    return true;
  return ac != 0 && ad != 0 && ca != 0 && cb != 0 &&
    ((ac < 0) != (ad < 0)) && ((ca < 0) != (cb < 0));
}

bool inside_triangle(Point2d p, const Triangle& t) {
  return turn(t[0],t[1],p) >= 0 && turn(t[1],t[2],p) >= 0 && turn(t[2],t[0],p) >= 0;
}

std::vector<Triangle> triangulate(CargoPolygon vertices) {
  if (vertices.size() > 3 && vertices.front().x == vertices.back().x &&
      vertices.front().y == vertices.back().y)
    vertices.pop_back();
  if (vertices.size() < 3) throw std::invalid_argument("cargo polygon needs at least three vertices");
  long double area = 0;
  for (std::size_t i=0; i<vertices.size(); ++i) {
    const auto a=vertices[i], b=vertices[(i+1)%vertices.size()];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || (a.x==b.x && a.y==b.y))
      throw std::invalid_argument("invalid cargo polygon vertex");
    area += static_cast<long double>(a.x)*b.y-static_cast<long double>(a.y)*b.x;
    for (std::size_t j=i+2; j<vertices.size(); ++j) {
      if (i==0 && j+1==vertices.size()) continue;
      if (segments_intersect(a,b,vertices[j],vertices[(j+1)%vertices.size()]))
        throw std::invalid_argument("cargo polygon edges intersect");
    }
  }
  if (area == 0) throw std::invalid_argument("cargo polygon has zero area");
  if (area < 0) std::reverse(vertices.begin(), vertices.end());
  std::vector<Triangle> triangles;
  while (vertices.size()>3) {
    bool found=false;
    for (std::size_t i=0; i<vertices.size(); ++i) {
      const std::size_t prev=(i+vertices.size()-1)%vertices.size(), next=(i+1)%vertices.size();
      const Triangle t{vertices[prev],vertices[i],vertices[next]};
      const auto bend=turn(t[0],t[1],t[2]);
      if (bend == 0 && on_segment(t[0],t[2],t[1])) {
        vertices.erase(vertices.begin()+i);
        found=true;
        break;
      }
      if (bend <= 0) continue;
      bool blocked=false;
      for (std::size_t j=0; j<vertices.size() && !blocked; ++j)
        if (j!=prev && j!=i && j!=next) blocked=inside_triangle(vertices[j],t);
      if (blocked) continue;
      triangles.push_back(t);
      vertices.erase(vertices.begin()+i);
      found=true;
      break;
    }
    if (!found) throw std::invalid_argument("unable to triangulate cargo polygon");
  }
  if (turn(vertices[0],vertices[1],vertices[2]) > 0)
    triangles.push_back({vertices[0],vertices[1],vertices[2]});
  if (triangles.empty()) throw std::invalid_argument("cargo polygon is degenerate");
  return triangles;
}

// The intersection of the convex triangle with a horizontal slab is convex.
// Its extreme x values are triangle vertices or edge/slab intersections.
std::pair<double,double> slab_extent(const Triangle& triangle, double lower, double upper) {
  double left=std::numeric_limits<double>::infinity(), right=-left;
  const auto include=[&](double x) {left=std::min(left,x); right=std::max(right,x);};
  for (std::size_t i=0; i<3; ++i) {
    const auto a=triangle[i], b=triangle[(i+1)%3];
    if (lower<=a.y && a.y<=upper) include(a.x);
    if (a.y==b.y) continue;
    for (double y:{lower,upper}) {
      if (std::min(a.y,b.y)<=y && y<=std::max(a.y,b.y))
        include(std::clamp(a.x+(y-a.y)/(b.y-a.y)*(b.x-a.x),std::min(a.x,b.x),std::max(a.x,b.x)));
    }
  }
  return {left,right};
}
} // namespace

FormationCollision::FormationCollision(const Request& r, const CargoPolygon& polygon)
: bodies_(*r.grid,r.vehicles,polygon.empty()?r.cargo:std::nullopt),origin_(r.grid->origin()),
  resolution_(r.grid->resolution()),right_(origin_.x+r.grid->width()*resolution_),
  top_(origin_.y+r.grid->height()*resolution_)
{
  if (polygon.empty()) return;
  triangles_=triangulate(polygon);
  for (const auto& p:polygon) radius_=std::max(radius_,std::hypot(p.x,p.y));
  if (!std::isfinite(radius_)) throw std::invalid_argument("invalid cargo polygon radius");

  occupied_rows_.resize(r.grid->height());
  for (std::size_t row=0; row<r.grid->height(); ++row)
    for (std::size_t col=0; col<r.grid->width(); ++col)
      if (r.grid->occupied(col,row)) occupied_rows_[row].push_back(col);
}

bool FormationCollision::polygon_clear(Pose2d reference, double padding) const {
  const fwis_planning_core::Transform2d transform(reference);
  // A square expansion covers the Euclidean swept displacement. The small
  // round-off allowance also keeps exact touches on either adjacent cell.
  const double scale=1+std::abs(origin_.x)+std::abs(origin_.y)+std::abs(right_)+std::abs(top_)+radius_;
  const double guard = padding+64*std::numeric_limits<double>::epsilon()*scale;
  const double half=resolution_/2+guard;
  for (const auto& local:triangles_) {
    Triangle triangle;
    std::transform(local.begin(),local.end(),triangle.begin(),[&](Point2d p){return transform.apply(p);});
    double bottom=top_, top=origin_.y;
    for (const auto& p:triangle) {
      if (!(origin_.x+guard < p.x && p.x < right_-guard &&
            origin_.y+guard < p.y && p.y < top_-guard)) return false;
      bottom=std::min(bottom,p.y);
      top=std::max(top,p.y);
    }
    const auto first=static_cast<std::size_t>(std::max(0.,std::floor((bottom-guard-origin_.y)/resolution_)-1));
    const auto last=static_cast<std::size_t>(std::min(double(occupied_rows_.size()-1),
        std::floor((top+guard-origin_.y)/resolution_)+1));
    for (std::size_t row=first; row<=last; ++row) {
      const auto& columns=occupied_rows_[row];
      if (columns.empty()) continue;
      const double center_y=origin_.y+(row+.5)*resolution_;
      const auto [left,right]=slab_extent(triangle,center_y-half,center_y+half);
      if (left>right) continue;
      const auto center_x=[&](std::size_t col){return origin_.x+(col+.5)*resolution_;};
      const auto cell=std::lower_bound(columns.begin(),columns.end(),left-half,
        [&](std::size_t col,double x){return center_x(col)<x;});
      if (cell!=columns.end() && center_x(*cell)<=right+half) return false;
    }
  }
  return true;
}

fwis_planning_core::CollisionResult FormationCollision::check(
    Pose2d reference, const double* headings, std::size_t count, double padding) const {
  if (!std::isfinite(padding) || padding<0) throw std::invalid_argument("invalid collision padding");
  if (!triangles_.empty() && !polygon_clear(reference,padding))
    return {fwis_planning_core::CollisionReason::cargo_map,0,0,1,0};
  auto result=bodies_.check(reference,headings,count,padding);
  result.map_queries+=!triangles_.empty();
  return result;
}

const char* FormationCollision::reason(fwis_planning_core::CollisionReason collision) const noexcept {
  if (!triangles_.empty() && collision==fwis_planning_core::CollisionReason::cargo_map)
    return "CARGO_POLYGON_COLLISION";
  return fwis_planning_core::reason_name(collision);
}
} // namespace fwis_hybrid_joint
