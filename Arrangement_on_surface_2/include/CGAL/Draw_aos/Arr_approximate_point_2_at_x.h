#ifndef CGAL_DRAW_AOS_ARR_APPROXIMATE_POINT_2_AT_X_H
#define CGAL_DRAW_AOS_ARR_APPROXIMATE_POINT_2_AT_X_H
#include "CGAL/Draw_aos/Arr_approximate_point_2.h"
#include "CGAL/Draw_aos/Arr_render_context.h"
#include "CGAL/Draw_aos/helpers.h"
#include "CGAL/Draw_aos/type_utils.h"
#include "CGAL/number_utils.h"
#include <boost/iterator/function_output_iterator.hpp>

namespace CGAL {
namespace draw_aos {

/**
 * @brief Functor to compute the point at a given x-coordinate on an x-monotone curve within a bounding box.
 */
class Arr_approximate_point_2_at_x
{
public:
  using Point_2 = Type_traits<Geom_traits>::Point_2;
  using X_monotone_curve_2 = Type_traits<Geom_traits>::X_monotone_curve_2;
  using Intersect_2 = Type_traits<Geom_traits>::Intersect_2;
  using Construct_min_vertex_2 = Type_traits<Geom_traits>::Construct_min_vertex_2;
  using FT = Type_traits<Geom_traits>::FT;
  using Is_vertical_2 = Type_traits<Geom_traits>::Is_vertical_2;
  using Approx_point = Arr_approximation_geometry_traits::Approx_point;

  Arr_approximate_point_2_at_x(const Arr_render_context& ctx)
      : m_ctx(ctx)
      , m_approx(ctx.traits) {}

  /**
   * @brief Computes the point at a given x-coordinate on an x-monotone curve trimmed
   * to the bounding box.
   *
   * The bounding box here is considered as closed.
   *
   * @precondition The curve is not verical
   * @param curve
   * @param x
   * @return true if there is an intersection at given x within the bounding box,
   * @return false otherwise.
   */
  std::optional<Approx_point> operator()(const X_monotone_curve_2& curve, FT x) const {
    CGAL_assertion(!m_ctx.is_vertical_2(curve));

    using Multiplicity = Geom_traits::Multiplicity;
    using Intersect_point = std::pair<Point_2, Multiplicity>;
    using Intersect_curve = X_monotone_curve_2;
    using Intersect_type = std::variant<Intersect_point, Intersect_curve>;

    auto vertical_line = m_ctx.cst_vertical_segment(x, m_ymin, m_ymax);
    std::optional<Approx_point> pt;
    auto func_out_iter = boost::make_function_output_iterator([&pt, this](const Intersect_type& res) {
      CGAL_assertion_msg(std::holds_alternative<Intersect_point>(res),
                         "Unexpected intersection type, expected Intersect_point");
      if(pt.has_value()) {
        return;
      }
      pt = this->m_approx(std::get<Intersect_point>(res).first);
    });
    m_ctx.intersect_2(vertical_line, curve, func_out_iter);

    if(pt.has_value()) {
      pt = Approx_point(CGAL::to_double(x), pt->y());
    }
    return pt;
  }

private:
  const Arr_approximate_point_2<Geom_traits> m_approx;
  const Arr_render_context& m_ctx;

  // Should be enough for visualization purposes.
  // constexpr static double m_ymin = std::numeric_limits<double>::lowest();
  // constexpr static double m_ymax = std::numeric_limits<double>::max();
  // maximum of double is too large for CORE number types, no idea why
  constexpr static double m_ymin = -1e8;
  constexpr static double m_ymax = 1e8;
};

} // namespace draw_aos
} // namespace CGAL

#endif // CGAL_DRAW_AOS_ARR_COMPUTE_Y_AT_X_H