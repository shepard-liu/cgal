#ifndef CGAL_DRAW_AOS_ARR_BOUNDED_APPROXIMATE_HALFEDGE_H
#define CGAL_DRAW_AOS_ARR_BOUNDED_APPROXIMATE_HALFEDGE_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <optional>

#include <boost/iterator/function_output_iterator.hpp>

#include <CGAL/Arr_enums.h>
#include <CGAL/Draw_aos/Arr_render_context.h>
#include <CGAL/Draw_aos/type_utils.h>
#include "CGAL/Draw_aos/Arr_projector.h"
#include "CGAL/Kernel/global_functions_3.h"

namespace CGAL {
namespace draw_aos {

/**
 * @brief Functor to approximate an x-monotone curve within an bounding box.
 *
 * The Approximation is done from xmin to xmax with a given step. For parts outbound the y limits and precedes or
 * succeeds a part within, the approximation may be skipped but there will be at least one point outside the bbox
 * for indication.
 */
template <typename Arrangement>
class Arr_bounded_approximate_halfedge
{
  using Geom_traits = typename Arrangement::Geometry_traits_2;
  using Halfedge_const_handle = typename Arrangement::Halfedge_const_handle;

  using Approx_traits = Arr_approximate_traits<Geom_traits>;
  using Approx_nt = typename Approx_traits::Approx_nt;
  using Approx_point = typename Approx_traits::Approx_point;
  using Point = typename Approx_traits::Point;
  using Polyline = typename Approx_traits::Polyline;
  using Approx_kernel = typename Approx_traits::Approx_kernel;
  using Approx_line_2 = typename Approx_kernel::Line_2;
  using X_monotone_curve_2 = typename Geom_traits::X_monotone_curve_2;
  using Bounded_render_context = Arr_bounded_render_context<Arrangement>;
  using Boundary_lines = std::array<Approx_line_2, 4>;

private:
  struct Context : public Bounded_render_context
  {
    Context(const Bounded_render_context& ctx,
            const X_monotone_curve_2& curve,
            Polyline& polyline,
            const Boundary_lines& boundary_lines)
        : Bounded_render_context(ctx)
        , m_curve(curve)
        , m_boundary_lines(boundary_lines)
        , m_proj(ctx.m_traits)
        , m_base_out_it(std::back_inserter(polyline))
        , m_out_it(boost::make_function_output_iterator(std::function([this](Point pt) {
          if(pt.x() < this->xmin()) {
            // We need the last point if not yet x-inbound.
            m_last_pt = pt;
            return;
          } else if(pt.x() > this->xmax())
            return;

          *m_base_out_it++ = pt;
          m_last_pt = pt;
        }))) {}

  private:
    std::back_insert_iterator<Polyline> m_base_out_it;

  public:
    Arr_projector<Geom_traits> m_proj;
    const X_monotone_curve_2& m_curve;
    const Boundary_lines& m_boundary_lines;
    std::optional<Point> m_last_pt;
    boost::function_output_iterator<std::function<void(Point)>> m_out_it;
  };

  static Point trace_boundary_inter(const Context& ctx, Point pt, Boundary_side side) {
    Point inter = std::get<Point>(
        *CGAL::intersection(Approx_line_2(*ctx.m_last_pt, pt), ctx.m_boundary_lines[static_cast<std::size_t>(side)]));
    // Prevent floating point errors.
    switch(side) {
    case Boundary_side::Left:
      return Point(ctx.xmin(), inter.y());
    case Boundary_side::Right:
      return Point(ctx.xmax(), inter.y());
    case Boundary_side::Top:
      return Point(inter.x(), ctx.ymax());
    case Boundary_side::Bottom:
      return Point(inter.x(), ctx.ymin());
    default:
      CGAL_assertion(false && "Unexpected side of boundary.");
      return Point();
    }
  }

  static void update(Context& ctx, Point pt) {
    if(!ctx.m_last_pt.has_value()) {
      *ctx.m_out_it++ = pt;
      return;
    }

    if(ctx.m_last_pt->x() < ctx.xmin() && pt.x() >= ctx.xmin()) {
      *ctx.m_out_it++ = trace_boundary_inter(ctx, pt, Boundary_side::Left);
    }

    if(ctx.m_last_pt->y() < ctx.ymin()) {
      if(pt.y() > ctx.ymin()) {
        *ctx.m_out_it++ = trace_boundary_inter(ctx, pt, Boundary_side::Bottom);
      }
      if(pt.y() > ctx.ymax()) {
        *ctx.m_out_it++ = trace_boundary_inter(ctx, pt, Boundary_side::Top);
      }
    } else if(ctx.m_last_pt->y() > ctx.ymax()) {
      if(pt.y() < ctx.ymax()) {
        *ctx.m_out_it++ = trace_boundary_inter(ctx, pt, Boundary_side::Top);
      }
      if(pt.y() < ctx.ymin()) {
        *ctx.m_out_it++ = trace_boundary_inter(ctx, pt, Boundary_side::Bottom);
      }
    } else {
      if(pt.y() < ctx.ymin()) {
        *ctx.m_out_it++ = trace_boundary_inter(ctx, pt, Boundary_side::Bottom);
      } else if(pt.y() > ctx.ymax()) {
        *ctx.m_out_it++ = trace_boundary_inter(ctx, pt, Boundary_side::Top);
      }
    }

    if(ctx.m_last_pt->x() <= ctx.xmax() && pt.x() > ctx.xmax()) {
      *ctx.m_out_it++ = trace_boundary_inter(ctx, pt, Boundary_side::Right);
    }

    *ctx.m_out_it++ = pt;
  }

public:
  Arr_bounded_approximate_halfedge(const Bounded_render_context& ctx)
      : m_ctx(ctx)
      , m_boundary_lines({
            Approx_line_2(Point(ctx.xmin(), ctx.ymax()), Point(ctx.xmax(), ctx.ymax())), // Top = 0
            Approx_line_2(Point(ctx.xmin(), ctx.ymin()), Point(ctx.xmin(), ctx.ymax())), // Left = 1
            Approx_line_2(Point(ctx.xmin(), ctx.ymin()), Point(ctx.xmax(), ctx.ymin())), // Bottom = 2
            Approx_line_2(Point(ctx.xmax(), ctx.ymin()), Point(ctx.xmax(), ctx.ymax())), // Right = 3
        }) {}

  const Polyline& operator()(const Halfedge_const_handle& he) const {
    CGAL_assertion(!he->is_fictitious());

    auto& cache = m_ctx.m_cache.halfedges();
    auto [iter, inserted] = cache.try_emplace(he, Polyline());
    Polyline& polyline = iter->second;
    if(!inserted) return polyline;
    if(m_ctx.is_cancelled()) return polyline;

    const X_monotone_curve_2& curve = he->curve();
    Context ctx(m_ctx, curve, polyline, m_boundary_lines);
    m_ctx.m_traits.approximate_2_object()(
        curve, m_ctx.m_approx_error,
        boost::make_function_output_iterator([&ctx](Approx_point pt) { update(ctx, ctx.m_proj.project(pt)); }),
        true); // ltr ordering
    Polyline poly_copy(polyline);
    transform_polyline(ctx, polyline, he);

    // also approximate the twin halfedge
    auto [twin_iter, twin_inserted] = cache.try_emplace(he->twin(), std::move(poly_copy));
    if(twin_inserted) transform_polyline(ctx, twin_iter->second, he->twin());

    // The previous iterator might have been invalidated by the second try_emplace call.
    return cache.at(he);
  }

  /**
   * @brief transform approximated curve points(ltr ordering) in place based on the halfedge, giving correct
   * ordering, continuity, etc.
   */
  static void transform_polyline(Context& ctx, Polyline& polyline, const Halfedge_const_handle& he) {
    transform_polyline_impl<Geom_traits>(ctx, polyline, he);
  }

  template <typename Gt, std::enable_if_t<!is_or_derived_from_curved_surf_traits_v<Gt>, int> = 0>
  static void transform_polyline_impl(Context& ctx, Polyline& polyline, const Halfedge_const_handle& he) {
    if(he->direction() == CGAL::ARR_LEFT_TO_RIGHT) return;
    std::reverse(polyline.begin(), polyline.end());
  }

  template <typename Gt, std::enable_if_t<is_or_derived_from_agas_v<Gt>, int> = 0>
  static void transform_polyline_impl(Context& ctx, Polyline& polyline, const Halfedge_const_handle& he) {
    using Direction_3 = typename Geom_traits::Direction_3;
    using Vector_3 = typename Geom_traits::Vector_3;

    if(polyline.size() < 2) return;
    const X_monotone_curve_2& curve = he->curve();
    const auto& traits = ctx.m_traits;
    if(curve.is_vertical()) {
      Direction_3 normal_dir = curve.is_directed_right() ? curve.normal() : -curve.normal();
      Direction_3 curve_dir(CGAL::cross_product(Vector_3(0, 0, 1), normal_dir.vector()));
      Approx_nt azimuth =
          ctx.m_proj.project(traits.approximate_2_object()(traits.construct_point_2_object()(curve_dir))).x();
      if(azimuth == 0 && he->direction() == ARR_LEFT_TO_RIGHT) azimuth = 2 * CGAL_PI;
      std::transform(polyline.begin(), polyline.end(), polyline.begin(),
                     [azimuth](Point pt) { return Point(azimuth, pt.y()); });
    } else if(polyline.back().x() == 0) {
      // For strictly x-monotone arcs whose target point sits on the boundary, the x should be set to 2 * CGAL_PI
      polyline.back() = Point(2 * CGAL_PI, polyline.back().y());
    }
    if(he->direction() == CGAL::ARR_LEFT_TO_RIGHT) return;
    std::reverse(polyline.begin(), polyline.end());
  }

private:
  const Bounded_render_context& m_ctx;
  const Boundary_lines m_boundary_lines;
};

} // namespace draw_aos
} // namespace CGAL
#endif