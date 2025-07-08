
#ifndef CGAL_DRAW_AOS_ARR_BOUNDED_APPROXIMATE_POINT_2_H
#define CGAL_DRAW_AOS_ARR_BOUNDED_APPROXIMATE_POINT_2_H

#include <utility>

#include <CGAL/Draw_aos/type_utils.h>
#include <CGAL/Draw_aos/Arr_render_context.h>
#include <CGAL/Draw_aos/helpers.h>
#include <CGAL/Draw_aos/Arr_approximate_point_2.h>
#include <CGAL/Draw_aos/Arr_approximation_geometry_traits.h>

namespace CGAL {
namespace draw_aos {

class Arr_bounded_approximate_point_2
{
  using Point_2 = Type_traits<Geom_traits>::Point_2;
  using Vertex_const_handle = Arrangement::Vertex_const_handle;
  using Point_geom = Arr_approximation_geometry_traits::Point_geom;

public:
  Arr_bounded_approximate_point_2(const Arr_bounded_render_context& ctx)
      : m_ctx(ctx) {}

  /**
   * @brief Approximate a vertex within the x-bounded range.
   *
   * The function uses cached values if available.
   * @precondition: The vertex must have an associated point.
   *
   * If the point is outside the bounding box, it returns Point_geom{} and false.
   * Otherwise, it returns the approximated point and true.
   * @param vh the vertex handle
   * @return std::pair<const Point_geom&, bool>
   */
  std::pair<const Point_geom&, bool> operator()(const Vertex_const_handle& vh) const {
    const auto& pt = vh->point();

    if(!m_ctx.strictly_contains(pt)) {
      return {Point_geom(), false};
    }

    auto [point, inserted] = m_ctx.cache.try_emplace(vh);
    if(!inserted) {
      return {point, true};
    }

    return {point = m_ctx.approx_pt(pt), true};
  }

private:
  const Arr_bounded_render_context& m_ctx;
};

} // namespace draw_aos
} // namespace CGAL

#endif // CGAL_DRAW_AOS_ARR_BOUNDED_APPROXIMATE_POINT_2_H
