#ifndef CGAL_DRAW_AOS_HELPERS_H
#define CGAL_DRAW_AOS_HELPERS_H
#include "CGAL/Algebraic_kernel_d_1.h"
#include "CGAL/Algebraic_kernel_for_circles_2_2.h"
#include "CGAL/Arr_circle_segment_traits_2.h"
#include "CGAL/Arr_circular_line_arc_traits_2.h"
#include "CGAL/Arr_conic_traits_2.h"
#include "CGAL/Arr_linear_traits_2.h"
#include "CGAL/Arr_rational_function_traits_2.h"
#include "CGAL/CORE_algebraic_number_traits.h"
#include "CGAL/Circular_kernel_2.h"
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
namespace CGAL {
namespace draw_aos {

using Exact_kernel = CGAL::Exact_predicates_exact_constructions_kernel;

// using Geom_traits = Arr_segment_traits_2<Exact_kernel>;
//
// using Geom_traits = CGAL::Arr_linear_traits_2<Exact_kernel>;

// using Nt_traits = CGAL::CORE_algebraic_number_traits;
// using Rational = Nt_traits::Rational;
// using Rat_kernel = CGAL::Cartesian<Rational>;
// using Alg_kernel = CGAL::Cartesian<CORE_algebraic_number_traits::Algebraic>;
// using Geom_traits = CGAL::Arr_conic_traits_2<Rat_kernel, Alg_kernel, Nt_traits>;

// using Geom_traits = CGAL::Arr_circle_segment_traits_2<Exact_kernel>;

// using NT = CGAL::Quotient<CGAL::MP_Float>;
// using Linear_k = CGAL::Cartesian<NT>;
// using Algebraic_k = CGAL::Algebraic_kernel_for_circles_2_2<NT>;
// using Circular_k = CGAL::Circular_kernel_2<Linear_k, Algebraic_k>;
// using Geom_traits = CGAL::Arr_circular_line_arc_traits_2<Circular_k>;

using AK1 = CGAL::Algebraic_kernel_d_1<CORE::BigInt>;
using Geom_traits = CGAL::Arr_rational_function_traits_2<AK1>;

using Arrangement = Arrangement_2<Geom_traits>;

struct Inner_ccb_tag
{};
struct Outer_ccb_tag
{};

} // namespace draw_aos
} // namespace CGAL

#endif // CGAL_DRAW_AOS_HELPERS_H