
#include "CGAL/number_type_config.h"
#define CGAL_DRAW_AOS_DEBUG
#define CGAL_DRAW_AOS_TRIANGULATOR_DEBUG_FILE_DIR "/Users/shep/Codes/aos_2_js_helper"
#include <vector>
#include "CGAL/Bbox_2.h"
#include "CGAL/Draw_aos/type_utils.h"
#include "arr_geodesic_on_sphere.h"
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arrangement_on_surface_2.h>
#include <CGAL/Arr_geodesic_arc_on_sphere_traits_2.h>
#include <CGAL/Arr_spherical_topology_traits_2.h>
#include <CGAL/Draw_aos/Arr_projector.h>
#include <CGAL/draw_arrangement_2.h>

#include "arr_geodesic.h"
#include "arr_print.h"

int main() {
  Geom_traits traits;
  auto ctr_p = traits.construct_point_2_object();
  auto ctr_cv = traits.construct_curve_2_object();
  Arrangement arr(&traits);

  CGAL::draw_aos::Arr_projector<Geom_traits> proj(traits);
  using Point_geom = CGAL::draw_aos::Arr_approximate_traits<Geom_traits>::Point_geom;
  std::vector<Point_geom> raw_poly1{
      {3, 3},   {3, 10},  {3, 17}, {2, 20}, {5, 30}, {34, 31}, {51, 31},
      {58, 24}, {61, 18}, {62, 8}, {62, 1}, {56, 1}, {25, 2},  {10, 2},
  };
  std::vector<Point_geom> raw_poly2{
      {13, 7}, {10, 9}, {9, 14}, {9, 20}, {9, 26}, {15, 26}, {22, 26}, {26, 19}, {26, 11}, {21, 8},
  };
  std::vector<Point_geom> raw_poly3{
      {40, 6}, {33, 7}, {33, 12}, {33, 20}, {45, 23}, {51, 23}, {54, 19}, {54, 9}, {53, 4},
  };
  std::vector<Point_geom> raw_poly4{
      {45, 10}, {39, 12}, {38, 16}, {45, 18}, {50, 16}, {48, 12},
  };

  std::vector<std::vector<Point_geom>> polygons{raw_poly1, raw_poly2, raw_poly3, raw_poly4};
  // locate the bounding box of all points
  CGAL::Bbox_2 bbox;
  for(auto poly : polygons) {
    for(const auto& pt : poly) {
      bbox += pt.bbox();
    }
  }
  // do affine transformation to fit the bounding box in [0.1, 1.9 Pi] [0.1, 0.9 Pi]
  double x_scale = (1.9 * CGAL_PI - 0.1) / bbox.xmax();
  double y_scale = (0.9 * CGAL_PI - 0.1) / bbox.ymax();
  double x_offset = 0.1 - bbox.xmin() * x_scale;
  double y_offset = 0.1 - bbox.ymin() * y_scale;
  auto transform = [x_scale, y_scale, x_offset, y_offset](Point_geom pt) {
    return Point_geom(pt.x() * x_scale + x_offset, pt.y() * y_scale + y_offset);
  };
  for(auto& poly : polygons) {
    std::transform(poly.begin(), poly.end(), poly.begin(), transform);
  }

  // insert polygons
  std::vector<Curve> arcs;
  for(const auto& poly : polygons) {
    for(size_t i = 0; i < poly.size(); ++i) {
      size_t next_i = (i + 1) % poly.size();
      auto from = proj.unproject(poly[i]);
      auto to = proj.unproject(poly[next_i]);
      arcs.push_back(ctr_cv(ctr_p(from.dx(), from.dy(), from.dz()), ctr_p(to.dx(), to.dy(), to.dz())));
    }
  }
  CGAL::insert(arr, arcs.begin(), arcs.end());

  print_arrangement_size(arr);
  CGAL::draw(arr, "spherical lakes");
  return 0;
}
