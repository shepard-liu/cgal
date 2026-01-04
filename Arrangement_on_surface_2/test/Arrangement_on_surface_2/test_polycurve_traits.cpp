#include <CGAL/Arr_polycurve_basic_traits_2.h>
#include <CGAL/Arr_polycurve_traits_2.h>
#include <CGAL/Arr_polyline_traits_2.h>

#include "CGAL/Arr_linear_traits_2.h"
#include "CGAL/Exact_predicates_exact_constructions_kernel.h"
#include "decorator_test.h"

int main() {
  Arr_polycurve_basic_traits_2<Arr_linear_traits_2<Exact_predicates_exact_constructions_kernel>> traits;
  auto cst_cv = traits.subcurve_traits_2()->construct_x_monotone_curve_2_object();
  auto cst_pt = traits.subcurve_traits_2()->construct_point_2_object();
  traits.parameter_space_in_x_2_object()(cst_cv(cst_pt(0, 0), cst_pt(1, 1)), ARR_MIN_END);
  traits.parameter_space_in_y_2_object()(cst_cv(cst_pt(0, 0), cst_pt(1, 1)), ARR_MIN_END);
}