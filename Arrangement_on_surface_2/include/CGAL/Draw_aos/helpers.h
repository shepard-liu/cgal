#ifndef CGAL_DRAW_AOS_HELPERS_H
#define CGAL_DRAW_AOS_HELPERS_H
#include "CGAL/Arr_circle_segment_traits_2.h"
#include "CGAL/Arr_conic_traits_2.h"
#include "CGAL/Arr_linear_traits_2.h"
#include "CGAL/CORE_algebraic_number_traits.h"
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

using Geom_traits = CGAL::Arr_circle_segment_traits_2<Exact_kernel>;

using Arrangement = Arrangement_2<Geom_traits>;

struct Inner_ccb_tag
{};
struct Outer_ccb_tag
{};

} // namespace draw_aos
} // namespace CGAL

#endif // CGAL_DRAW_AOS_HELPERS_H