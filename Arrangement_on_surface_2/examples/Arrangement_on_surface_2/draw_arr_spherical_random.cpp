#include <array>
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

  std::array<Curve, 110> curves;
  CGAL::Random random;
  for(int i = 0; i < 100; ++i) {
    double x1 = random.get_double(-1.0, 1.0);
    double y1 = random.get_double(-1.0, 1.0);
    double z1 = random.get_double(-1.0, 1.0);
    double x2 = random.get_double(-1.0, 1.0);
    double y2 = random.get_double(-1.0, 1.0);
    double z2 = random.get_double(-1.0, 1.0);

    Point_2 p1 = ctr_p(x1, y1, z1);
    Point_2 p2 = ctr_p(x2, y2, z2);
    curves[i] = ctr_cv(p1, p2);
  }

  // generate 10 vertical curves
  for(int i = 0; i < 10; ++i) {
    double x = random.get_double(-1.0, 1.0);
    double y = random.get_double(-1.0, 1.0);
    double z1 = random.get_double(-1.0, 1.0);
    double z2 = random.get_double(-1.0, 1.0);

    Point_2 p1 = ctr_p(x, y, z1);
    Point_2 p2 = ctr_p(x, y, z2);
    curves[i + 100] = ctr_cv(p1, p2);
  }

  CGAL::insert(arr, curves.begin(), curves.end());

  using Epick = CGAL::Exact_predicates_inexact_constructions_kernel;
  CGAL::Constrained_Delaunay_triangulation_2<Epick> ct;

  print_arrangement_size(arr);
  CGAL::draw(arr, "random arcs");
  return 0;
}
