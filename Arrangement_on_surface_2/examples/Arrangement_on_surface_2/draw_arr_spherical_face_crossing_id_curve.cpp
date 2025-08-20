#include <fstream>
#include <vector>
#include "arr_geodesic_on_sphere.h"
#define CGAL_DRAW_AOS_DEBUG
#define CGAL_DRAW_AOS_TRIANGULATOR_DEBUG_FILE_DIR "/Users/shep/Codes/aos_2_js_helper"
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arrangement_on_surface_2.h>
#include <CGAL/Arr_geodesic_arc_on_sphere_traits_2.h>
#include <CGAL/Arr_spherical_topology_traits_2.h>
#include <CGAL/draw_arrangement_2.h>

#include "arr_geodesic.h"
#include "arr_print.h"

int main() {
  Geom_traits traits;
  auto ctr_p = traits.construct_point_2_object();
  auto ctr_cv = traits.construct_curve_2_object();
  Arrangement arr(&traits);

  using X_monotone_curve_2 = typename Geom_traits::X_monotone_curve_2;
  using Direction_3 = typename Geom_traits::Direction_3;
  using Point_2 = typename Geom_traits::Point_2;

  Point p1 = ctr_p(-0.95, 0.32, 0), p2 = ctr_p(-0.87, 0.02, 0.49), p3 = ctr_p(-0.93, -0.36, 0),
        p4 = ctr_p(-0.81, -0.03, -0.59);
  auto arcs = {ctr_cv(p1, p2), ctr_cv(p2, p3), ctr_cv(p3, p4), ctr_cv(p4, p1)};
  CGAL::insert(arr, arcs.begin(), arcs.end());

  std::ofstream ofs("spherical_face_crossing_id_curve.txt");
  ofs << arr << std::endl;

  print_arrangement_size(arr);
  CGAL::draw(arr, "spherical face crossing identity curve");
  return 0;
}
