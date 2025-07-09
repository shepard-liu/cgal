#ifndef CGAL_DRAW_AOS_ARR_CONSTRUCT_SEGMENTS_H
#define CGAL_DRAW_AOS_ARR_CONSTRUCT_SEGMENTS_H
#include "CGAL/Arr_circle_segment_traits_2.h"
#include "CGAL/Arr_circular_line_arc_traits_2.h"
#include "CGAL/Draw_aos/helpers.h"
#include "CGAL/Draw_aos/type_utils.h"
#include "CGAL/number_utils.h"
#include <limits>
namespace CGAL {
namespace draw_aos {

template <typename GeomTraits, bool HasConstructXMonotoneCurve2>
class Arr_construct_segment_impl;

// Default implementation for traits that models Construct_x_monotone_curve_2
template <typename GeomTraits>
class Arr_construct_segment_impl<GeomTraits, true>
{
  using Point_2 = typename Type_traits<GeomTraits>::Point_2;
  using X_monotone_curve_2 = typename Type_traits<GeomTraits>::X_monotone_curve_2;
  using Construct_x_monotone_curve_2 = typename GeomTraits::Construct_x_monotone_curve_2;
  using FT = typename Type_traits<GeomTraits>::FT;

public:
  Arr_construct_segment_impl(const GeomTraits& traits)
      : m_cst_x_curve(traits.construct_x_monotone_curve_2_object()) {}

  X_monotone_curve_2 operator()(FT x1, FT y1, FT x2, FT y2) const {
    return m_cst_x_curve(Point_2(x1, y1), Point_2(x2, y2));
  }

private:
  const Construct_x_monotone_curve_2 m_cst_x_curve;
};

// Specialization for Arr_circle_segment_traits_2
template <typename Kernel>
class Arr_construct_segment_impl<Arr_circle_segment_traits_2<Kernel>, false>
{
  using Geom_traits = Arr_circle_segment_traits_2<Kernel>;
  using Point_2 = typename Type_traits<Geom_traits>::Point_2;
  using X_monotone_curve_2 = typename Type_traits<Geom_traits>::X_monotone_curve_2;
  using Line_2 = typename X_monotone_curve_2::Line_2;
  using Approximate_2 = typename Geom_traits::Approximate_2;
  using NT = typename Type_traits<Geom_traits>::FT;
  using FT = typename Kernel::FT;

private:
  static FT to_ft(NT v) { return v.a0() + v.a1() * CGAL::sqrt(v.root()); }

public:
  Arr_construct_segment_impl(const Geom_traits& traits) {}

  X_monotone_curve_2 operator()(NT x1, NT y1, NT x2, NT y2) const {
    using Kernel_point_2 = typename Kernel::Point_2;
    FT kx1 = to_ft(x1), ky1 = to_ft(y1), kx2 = to_ft(x2), ky2 = to_ft(y2);
    return X_monotone_curve_2(Kernel_point_2(kx1, ky1), Kernel_point_2(kx2, ky2));
  }
};

template <typename GeomTraits>
class Arr_construct_segment_impl<GeomTraits, false>
{
  static_assert(false, "Not implemented yet!");
};

template <typename GeomTraits>
using Arr_construct_segment =
    Arr_construct_segment_impl<GeomTraits, has_construct_x_monotone_curve_2<GeomTraits>::value>;

template <typename GeomTraits>
class Arr_construct_vertical_segment
{
  using Point_2 = typename Type_traits<GeomTraits>::Point_2;
  using X_monotone_curve_2 = typename Type_traits<GeomTraits>::X_monotone_curve_2;
  using FT = typename Type_traits<GeomTraits>::FT;

public:
  Arr_construct_vertical_segment(const GeomTraits& traits)
      : m_cst_seg(traits) {}

  X_monotone_curve_2 operator()(FT x, FT ymin, FT ymax) const { return m_cst_seg(x, ymin, x, ymax); }

private:
  const Arr_construct_segment<GeomTraits> m_cst_seg;
};

template <typename GeomTraits>
class Arr_construct_horizontal_segment
{
  using Point_2 = typename Type_traits<GeomTraits>::Point_2;
  using X_monotone_curve_2 = typename Type_traits<GeomTraits>::X_monotone_curve_2;
  using FT = typename Type_traits<GeomTraits>::FT;

public:
  Arr_construct_horizontal_segment(const GeomTraits& traits)
      : m_cst_seg(traits) {}

  X_monotone_curve_2 operator()(FT y, FT xmin, FT xmax) const { return m_cst_seg(xmin, y, xmax, y); }

private:
  const Arr_construct_segment<GeomTraits> m_cst_seg;
};

// Arr_construct_vertical_segment Specialization for Arr_rational_function_traits_2
template <typename Kernel>
class Arr_construct_vertical_segment<Arr_rational_function_traits_2<Kernel>>
{
  using Geom_traits = Arr_rational_function_traits_2<Kernel>;
  using Point_2 = typename Type_traits<Geom_traits>::Point_2;
  using X_monotone_curve_2 = typename Type_traits<Geom_traits>::X_monotone_curve_2;
  using FT = typename Type_traits<Geom_traits>::FT;
  using Polynomial_1 = typename Geom_traits::Polynomial_1;

public:
  Arr_construct_vertical_segment(const Geom_traits&) {}

  X_monotone_curve_2 operator()(FT x0, FT ymin, FT ymax) const {
    // We could only construct a near vertical segment
    Polynomial_1 x(0, 1);
    FT k = 1e8;
    FT xmin = ymin / k + x0;
    FT xmax = ymax / k + x0;
    return X_monotone_curve_2(k * (x - x0), xmin, xmax);
  }
};

// Arr_construct_horizontal_segment Specialization for Arr_rational_function_traits_2
template <typename Kernel>
class Arr_construct_horizontal_segment<Arr_rational_function_traits_2<Kernel>>
{
  using Geom_traits = Arr_rational_function_traits_2<Kernel>;
  using Point_2 = typename Type_traits<Geom_traits>::Point_2;
  using X_monotone_curve_2 = typename Type_traits<Geom_traits>::X_monotone_curve_2;
  using FT = typename Type_traits<Geom_traits>::FT;
  using Polynomial_1 = typename Geom_traits::Polynomial_1;

public:
  Arr_construct_horizontal_segment(const Geom_traits&) {}

  X_monotone_curve_2 operator()(FT y, FT xmin, FT xmax) const {
    Polynomial_1 p = y;
    return X_monotone_curve_2(p, xmin, xmax);
  }
};

} // namespace draw_aos
} // namespace CGAL

#endif