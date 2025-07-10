#ifndef CGAL_DRAW_AOS_ARR_BOUNDED_APPROXIMATE_CURVE_2_H
#define CGAL_DRAW_AOS_ARR_BOUNDED_APPROXIMATE_CURVE_2_H

#include "CGAL/Arr_enums.h"
#include "CGAL/Draw_aos/Arr_approximate_point_2_at_x.h"
#include "CGAL/Draw_aos/Arr_construct_curve_end.h"
#include "CGAL/Draw_aos/Arr_construct_segments.h"
#include "CGAL/Draw_aos/Arr_render_context.h"
#include "CGAL/Draw_aos/helpers.h"
#include "CGAL/Draw_aos/Arr_approximation_geometry_traits.h"
#include "CGAL/Draw_aos/type_utils.h"
#include "CGAL/basic.h"
#include <algorithm>
#include <cstddef>
#include <iterator>
#include <optional>
#include <vector>

namespace CGAL {
namespace draw_aos {

/**
 * @brief Functor to approximate an x-monotone curve within an bounding box.
 * The bbox here has closed boundary.
 *
 * The Approximation is done from xmin to xmax with a given step. For parts outbound the y limits and precedes or
 * succeeds a part within, the approximation may be skipped but there will be at least one point outside the bbox for
 * indication.
 *
 * TODO: Possible optimizations:
 * - Specialize for traits that models Approximate_2 on curves.
 */
class Arr_bounded_approximate_curve_2
{
  using FT = typename Type_traits<Geom_traits>::FT;
  using Point_2 = Type_traits<Geom_traits>::Point_2;
  using X_monotone_curve_2 = Type_traits<Geom_traits>::X_monotone_curve_2;
  using Halfedge_const_handle = Arrangement::Halfedge_const_iterator;
  using Approx_point = Arr_approximation_geometry_traits::Approx_point;
  using Point_geom = Arr_approximation_geometry_traits::Point_geom;
  using Polyline_geom = Arr_approximation_geometry_traits::Polyline_geom;
  using Intersections_vector = std::vector<Point_2>;

  struct Execution_context : public Arr_context_delegator<Arr_bounded_render_context>
  {
    Execution_context(const Arr_bounded_render_context& ctx,
                      const X_monotone_curve_2& curve,
                      const Arr_approximate_point_2_at_x& approx_pt_at_x,
                      const Intersections_vector& top_inters,
                      const Intersections_vector& bottom_inters,
                      Polyline_geom& polyline)
        : Arr_context_delegator(ctx)
        , curve(curve)
        , m_approx_pt_at_x(approx_pt_at_x)
        , top_inters(top_inters)
        , bottom_inters(bottom_inters)
        , out_it(std::back_inserter(polyline)) {
      auto min_end_pt = Arr_construct_curve_end<Geom_traits>(ctx.traits)(curve, ARR_MIN_END);
      if(min_end_pt.has_value()) {
        min_end = (*this)->approx_pt(min_end_pt.value());
      }
      auto max_end_pt = Arr_construct_curve_end<Geom_traits>(ctx.traits)(curve, ARR_MAX_END);
      if(max_end_pt.has_value()) {
        max_end = (*this)->approx_pt(max_end_pt.value());
      }
      txmin = is_min_end_bounded() ? std::clamp(min_end->x(), ctx.xmin(), ctx.xmax()) : ctx.xmin();
      txmax = is_max_end_bounded() ? std::clamp(max_end->x(), ctx.xmin(), ctx.xmax()) : ctx.xmax();
      tymin = is_min_end_bounded() ? std::clamp(min_end->y(), ctx.ymin(), ctx.ymax()) : ctx.ymin();
      tymax = is_max_end_bounded() ? std::clamp(max_end->y(), ctx.ymin(), ctx.ymax()) : ctx.ymax();
    }

    bool has_y_intersections() const { return !top_inters.empty() || !bottom_inters.empty(); }
    bool is_min_end_bounded() const { return min_end.has_value(); }
    bool is_max_end_bounded() const { return max_end.has_value(); }
    bool is_bounded_curve() const { return is_min_end_bounded() && is_max_end_bounded(); }
    std::optional<Approx_point> approx_pt_at_x(double x) const {
      if(x == txmin && is_min_end_bounded() && (*this)->contains_x(min_end->x())) {
        return min_end;
      }
      if(x == txmax && is_max_end_bounded() && (*this)->contains_x(max_end->x())) {
        return max_end;
      }
      return m_approx_pt_at_x(curve, to_ft(x));
    }

    const X_monotone_curve_2& curve;
    const Intersections_vector &top_inters, bottom_inters;
    std::optional<Approx_point> min_end, max_end;
    double txmin, txmax, tymin, tymax;
    std::back_insert_iterator<Polyline_geom> out_it;
    const Construct_coordinate<Geom_traits> to_ft;

  private:
    const Arr_approximate_point_2_at_x& m_approx_pt_at_x;
  };

private:
  static std::vector<Point_2> compute_intersections(const X_monotone_curve_2& cv1,
                                                    const X_monotone_curve_2& cv2,
                                                    const typename Geom_traits::Intersect_2& intersect_2,
                                                    const Arr_construct_curve_end<Geom_traits>& cst_curve_end) {
    using Intersect_point = std::pair<Point_2, Geom_traits::Multiplicity>;
    using Intersect_curve = X_monotone_curve_2;
    using Intersect_type = std::variant<Intersect_point, Intersect_curve>;

    std::vector<Point_2> intersections;
    auto out_it = std::back_inserter(intersections);
    intersect_2(cv1, cv2, boost::make_function_output_iterator([&out_it, &cst_curve_end](const Intersect_type& res) {
                  if(auto* pt = std::get_if<Intersect_point>(&res)) {
                    *out_it++ = pt->first;
                    return;
                  }
                  if(auto* cv = std::get_if<Intersect_curve>(&res)) {
                    *out_it++ = cst_curve_end(*cv, ARR_MIN_END).value();
                    *out_it++ = cst_curve_end(*cv, ARR_MAX_END).value();
                    return;
                  }
                  CGAL_assertion(false && "Unexpected intersection type");
                }));
    return intersections;
  }

  static std::optional<Approx_point> first_intersection(Execution_context& ctx) {
    if(!ctx.top_inters.empty() && !ctx.bottom_inters.empty()) {
      return ctx->approx_pt(ctx->compare_xy_2(ctx.top_inters.front(), ctx.bottom_inters.front()) == CGAL::SMALLER
                                ? ctx.top_inters.front()
                                : ctx.bottom_inters.front());
    }
    if(!ctx.top_inters.empty()) {
      return ctx->approx_pt(ctx.top_inters.front());
    }
    if(!ctx.bottom_inters.empty()) {
      return ctx->approx_pt(ctx.bottom_inters.front());
    }
    return std::nullopt;
  }

  /**
   * @brief approximate strictly x-monotone curve segment that does not cross the y bounds.
   *
   * @precondition: The segment is either inbound or outbound the bbox in the given range.
   * @param start the x-coordinate of the segment start(exclusive)
   * @param end the x-coordinate of the segment end(exclusive)
   * @param step the step to approximate the curve segment, negative values allowed.
   * @returns true if this part of the curve is within the closed bbox
   */
  static void approximate_simple_curve_segment(Execution_context& ctx, double start, double end, double step) {
    for(double x = start + step; x <= end - step; x += step) {
      auto pt = ctx.approx_pt_at_x(x);
      if(!pt.has_value()) {
        // break as soon as there's no more intersections
        break;
      }
      if(pt->y() == ctx->ymin() || pt->y() == ctx->ymax()) {
        // The segment overlaps with the bbox edge. There's no need to insert a dummy point.
        break;
      }
      *ctx.out_it++ = pt.value();
      if(!ctx->contains_y(pt->y())) {
        // We are outside the bbox. The dummy point was already inserted to indicate that.
        break;
      }
    }
  };

  static void approximate_vertical_curve(Execution_context& ctx) {
    if(ctx.is_bounded_curve() && !ctx->contains_x(ctx.min_end->x())) {
      // The curve is outside the bbox in x direction, no need to approximate
      return;
    }
    if(!ctx.is_bounded_curve() && !ctx.has_y_intersections()) {
      // The curve has unbounded end and has no intersections with the top or bottom edges,
      // it must be outbound in x direction.
      return;
    }
    // The vertical curve is now within the x bounds.

    if(ctx.tymax == ctx.tymin) {
      // But the curve is not degenerate. So, either it has only one point within the bbox or it is
      // entirely outside the bbox in y direction.
      return;
    }
    // Now we gaurantee that the curve has at least two points within the bbox in y direction.
    // We have to obtain the x coordinate of this vertical curve.
    double x = ctx.is_bounded_curve() ? ctx.min_end->x() : first_intersection(ctx).value().x();
    *ctx.out_it++ = Approx_point(x, ctx.tymin);
    *ctx.out_it++ = Approx_point(x, ctx.tymax);
  }

public:
  Arr_bounded_approximate_curve_2(const Arr_bounded_render_context& ctx)
      : to_ft()
      , m_ctx(ctx)
      , m_approx_pt_at_x(ctx)
      , m_top(ctx.cst_horizontal_segment(to_ft(ctx.ymax()), to_ft(ctx.xmin()), to_ft(ctx.xmax())))
      , m_bottom(ctx.cst_horizontal_segment(to_ft(ctx.ymin()), to_ft(ctx.xmin()), to_ft(ctx.xmax()))) {}

  /**
   * @brief Approximate an x-monotone curve from left to right within the bounding box.
   *
   * @param he non-fictitious halfedge handle
   * @return const Polyline_geom&
   */
  const Polyline_geom& operator()(const Halfedge_const_handle& he) const {
    CGAL_assertion(!he->is_fictitious());

    auto [polyline, inserted] = m_ctx.cache.try_emplace(he);
    if(!inserted) {
      return polyline;
    }

    if(m_ctx.is_cancelled()) {
      return polyline;
    }

    const X_monotone_curve_2& curve = he->curve();

    auto top_inters = compute_intersections(m_top, curve, m_ctx.intersect_2, m_ctx.cst_curve_end);
    auto bottom_inters = compute_intersections(m_bottom, curve, m_ctx.intersect_2, m_ctx.cst_curve_end);
    Execution_context ctx(m_ctx, curve, m_approx_pt_at_x, top_inters, bottom_inters, polyline);

    if(ctx->is_vertical_2(curve)) {
      approximate_vertical_curve(ctx);
      return polyline;
    }

    polyline.reserve(top_inters.size() + bottom_inters.size());

    double last_x;
    std::optional<Approx_point> first_inter = first_intersection(ctx);

    if(auto pt_at_txmin = ctx.approx_pt_at_x(ctx.txmin);
       pt_at_txmin.has_value() && pt_at_txmin->y() != ctx->ymin() && pt_at_txmin->y() != ctx->ymax())
    {
      // The tight starting point of the curve is within the bbox and
      // it's not on the top or bottom edge.
      *ctx.out_it++ = ctx.txmin == ctx->xmin() ? ctx->make_on_boundary(pt_at_txmin.value()) : pt_at_txmin.value();
      double segment_end = first_inter.has_value() ? first_inter->x() : ctx.txmax;
      approximate_simple_curve_segment(ctx, ctx.txmin, segment_end, ctx->approx_error);
      last_x = segment_end;
    } else if(first_inter.has_value()) {
      last_x = first_inter->x();
    } else {
      return polyline; // The curve is entirely outside the bbox in x direction.
    }

    // iterate through the intersections and insert segments in-between.
    std::merge(top_inters.begin(), top_inters.end(), bottom_inters.begin(), bottom_inters.end(),
               boost::make_function_output_iterator([&last_x, &ctx](const Point_2& inter) {
                 auto approx_inter = ctx->approx_pt(inter);
                 approximate_simple_curve_segment(ctx, last_x, approx_inter.x(), ctx->approx_error);
                 *ctx.out_it++ = ctx->make_on_boundary(approx_inter);
                 last_x = approx_inter.x();
               }),
               [&ctx](const Point_2& pt1, const Point_2& pt2) { return ctx->compare_xy_2(pt1, pt2) == CGAL::SMALLER; });

    if(auto pt_at_txmax = ctx.approx_pt_at_x(ctx.txmax);
       pt_at_txmax.has_value() && pt_at_txmax->y() != ctx->ymin() && pt_at_txmax->y() != ctx->ymax())
    {
      approximate_simple_curve_segment(ctx, last_x, ctx.txmax, ctx->approx_error);
      *ctx.out_it++ = ctx.txmax == ctx->xmax() ? ctx->make_on_boundary(pt_at_txmax.value()) : pt_at_txmax.value();
    }

    std::cout << "Approximate curve: " << std::endl;
    for(const auto& pt : polyline) {
      std::cout << "Point: (" << pt.x() << ", " << pt.y() << ")" << std::endl;
    }

    return polyline;
  }

private:
  const Construct_coordinate<Geom_traits> to_ft;
  const Arr_bounded_render_context& m_ctx;
  const Arr_approximate_point_2_at_x m_approx_pt_at_x;
  const X_monotone_curve_2 m_top;
  const X_monotone_curve_2 m_bottom;
};

} // namespace draw_aos
} // namespace CGAL
#endif // CGAL_DRAW_AOS_ARR_BOUNDED_APPROXIMATE_CURVE_2_H