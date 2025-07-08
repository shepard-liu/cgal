#ifndef CGAL_DRAW_AOS_ARR_CONSTRUCT_SEGMENTS_H
#define CGAL_DRAW_AOS_ARR_CONSTRUCT_SEGMENTS_H
#include "CGAL/Arr_circle_segment_traits_2.h"
#include "CGAL/Draw_aos/helpers.h"
#include "CGAL/Draw_aos/type_utils.h"
namespace CGAL {
namespace draw_aos {

template <typename GeomTraits, bool HasConstructXMonotoneCurve2>
class Arr_construct_segment_impl;

// Specialization for traits that have Construct_x_monotone_curve_2
template <typename GeomTraits>
class Arr_construct_segment_impl<GeomTraits, true>
{
  using Point_2 = typename Type_traits<GeomTraits>::Point_2;
  using X_monotone_curve_2 = typename Type_traits<GeomTraits>::X_monotone_curve_2;
  using Construct_x_monotone_curve_2 = typename GeomTraits::Construct_x_monotone_curve_2;

public:
  Arr_construct_segment_impl(const GeomTraits& traits)
      : m_cst_x_curve(traits) {}

  X_monotone_curve_2 operator()(const Point_2& p1, const Point_2& p2) const { return m_cst_x_curve(p1, p2); }

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

public:
  Arr_construct_segment_impl(const Geom_traits& traits)
      : m_approximate_2(traits.approximate_2_object()) {}

  X_monotone_curve_2 operator()(const Point_2& p1, const Point_2& p2) const {
    using Kernel_point_2 = typename Kernel::Point_2;
    /**
     * This looks weird.
     * I'm actually looking for a way to convert Point_2(_One_root_point_2 defined in Arr_circle_segment_traits_2) to
     * Kernel::Point_2 but failed to find one.
     *
     * Note: Don't use the constructed x_monotone_curve_2 for any topological computations.
     */
    auto x1 = m_approximate_2(p1, 0);
    auto y1 = m_approximate_2(p1, 1);
    auto x2 = m_approximate_2(p2, 0);
    auto y2 = m_approximate_2(p2, 1);
    return X_monotone_curve_2(Kernel_point_2(x1, y1), Kernel_point_2(x2, y2));
  }

private:
  Approximate_2 m_approximate_2;
};

template <typename GeomTraits>
class Arr_construct_segment_impl<GeomTraits, false>
{
  static_assert(false, "Not implemented yet!");
};

template <typename GeomTraits>
using Arr_construct_segment =
    Arr_construct_segment_impl<GeomTraits, has_construct_x_monotone_curve_2<GeomTraits>::value>;

class Arr_construct_vertical_segment
{
  using Point_2 = typename Type_traits<Geom_traits>::Point_2;
  using X_monotone_curve_2 = typename Type_traits<Geom_traits>::X_monotone_curve_2;
  using FT = typename Type_traits<Geom_traits>::FT;

public:
  Arr_construct_vertical_segment(const Geom_traits& traits)
      : m_cst_seg(traits) {}

  X_monotone_curve_2 operator()(FT x, FT ymin, FT ymax) const { return m_cst_seg(Point_2(x, ymin), Point_2(x, ymax)); }

private:
  const Arr_construct_segment<Geom_traits> m_cst_seg;
};

class Arr_construct_horizontal_segment
{
  using Point_2 = Type_traits<Geom_traits>::Point_2;
  using X_monotone_curve_2 = Type_traits<Geom_traits>::X_monotone_curve_2;
  using FT = Type_traits<Geom_traits>::FT;

public:
  Arr_construct_horizontal_segment(const Geom_traits& traits)
      : m_cst_seg(traits) {}

  X_monotone_curve_2 operator()(FT y, FT xmin, FT xmax) const { return m_cst_seg(Point_2(xmin, y), Point_2(xmax, y)); }

private:
  const Arr_construct_segment<Geom_traits> m_cst_seg;
};

} // namespace draw_aos
} // namespace CGAL

#endif