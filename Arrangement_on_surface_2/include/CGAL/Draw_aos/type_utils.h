#ifndef CGAL_DRAW_AOS_TYPE_UTILS_H
#define CGAL_DRAW_AOS_TYPE_UTILS_H
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_linear_traits_2.h>
#include <CGAL/Arr_circle_segment_traits_2.h>
#include <CGAL/Arr_conic_traits_2.h>
#include <CGAL/Arr_circular_arc_traits_2.h>
#include <CGAL/Arr_Bezier_curve_traits_2.h>
#include <CGAL/Arr_geodesic_arc_on_sphere_traits_2.h>
#include <CGAL/Arr_rational_function_traits_2.h>
#include <CGAL/Arr_algebraic_segment_traits_2.h>

namespace CGAL {
namespace draw_aos {

template <typename GeomTraits>
struct Type_traits;

template <typename Kernel>
struct Type_traits<Arr_segment_traits_2<Kernel>>
{
private:
  using Geom_traits = Arr_segment_traits_2<Kernel>;

public:
  constexpr static bool Has_unbounded_curves = false;
  using FT = typename Kernel::FT;
  using Point_2 = typename Kernel::Point_2;
  using X_monotone_curve_2 = typename Geom_traits::X_monotone_curve_2;
  using Intersect_2 = typename Geom_traits::Intersect_2;
  using Construct_min_vertex_2 = typename Geom_traits::Construct_min_vertex_2;
  using Approximate_2 = typename Geom_traits::Approximate_2;
  using Is_vertical_2 = typename Geom_traits::Is_vertical_2;
  using Compare_xy_2 = typename Geom_traits::Compare_xy_2;
};

template <typename Kernel>
struct Type_traits<Arr_linear_traits_2<Kernel>>
{
private:
  using Geom_traits = Arr_linear_traits_2<Kernel>;

public:
  constexpr static bool Has_unbounded_curves = true;
  using FT = typename Kernel::FT;
  using Point_2 = typename Kernel::Point_2;
  using X_monotone_curve_2 = typename Geom_traits::X_monotone_curve_2;
  using Intersect_2 = typename Geom_traits::Intersect_2;
  using Construct_min_vertex_2 = typename Geom_traits::Construct_min_vertex_2;
  using Approximate_2 = typename Geom_traits::Approximate_2;
  using Is_vertical_2 = typename Geom_traits::Is_vertical_2;
  using Compare_xy_2 = typename Geom_traits::Compare_xy_2;
};

template <typename RatKernel, typename AlgKernel, typename NtTraits>
struct Type_traits<Arr_conic_traits_2<RatKernel, AlgKernel, NtTraits>>
{
private:
  using Geom_traits = Arr_conic_traits_2<RatKernel, AlgKernel, NtTraits>;

public:
  constexpr static bool Has_unbounded_curves = false;
  using FT = typename AlgKernel::FT;
  using Point_2 = typename AlgKernel::Point_2;
  using X_monotone_curve_2 = typename Geom_traits::X_monotone_curve_2;
  using Intersect_2 = typename Geom_traits::Intersect_2;
  using Construct_min_vertex_2 = typename Geom_traits::Construct_min_vertex_2;
  using Approximate_2 = typename Geom_traits::Approximate_2;
  using Is_vertical_2 = typename Geom_traits::Is_vertical_2;
  using Compare_xy_2 = typename Geom_traits::Compare_xy_2;
};

} // namespace draw_aos
} // namespace CGAL

#endif // CGAL_DRAW_AOS_TYPE_UTILS_H