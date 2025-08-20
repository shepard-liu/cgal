#ifndef CGAL_DRAW_AOS_ARR_BOUNDED_APPROXIMATE_FACE_H
#define CGAL_DRAW_AOS_ARR_BOUNDED_APPROXIMATE_FACE_H

#include <cstddef>
#include <optional>
#include <utility>
#include <algorithm>

#include <boost/iterator/function_output_iterator.hpp>

#include <CGAL/Arr_enums.h>
#include <CGAL/Bbox_2.h>
#include "CGAL/basic.h"
#include <CGAL/Draw_aos/Arr_bounded_approximate_halfedge.h>
#include <CGAL/Draw_aos/Arr_bounded_approximate_vertex.h>
#include <CGAL/Draw_aos/Arr_projector.h>
#include <CGAL/Draw_aos/Arr_bounded_face_triangulator.h>
#include <CGAL/Draw_aos/Arr_render_context.h>
#include <CGAL/Draw_aos/type_utils.h>

namespace CGAL {

namespace draw_aos {

/*!
 * \brief Functor to approximate arrangement face with triangles within a bounding box.
 *
 * \tparam Arrangement
 */
template <typename Arrangement>
class Arr_bounded_approximate_face
{
  using Face_const_handle = typename Arrangement::Face_const_handle;
  using Halfedge_const_handle = typename Arrangement::Halfedge_const_handle;
  using Vertex_const_handle = typename Arrangement::Vertex_const_handle;
  using Ccb_halfedge_const_circulator = typename Arrangement::Ccb_halfedge_const_circulator;

  using Geom_traits = typename Arrangement::Geometry_traits_2;
  using Approx_traits = Arr_approximate_traits<Geom_traits>;
  using Point = typename Approx_traits::Point;
  using Polyline = typename Approx_traits::Polyline;
  using Triangle_soup = typename Approx_traits::Triangle_soup;

  using Bounded_approximate_vertex = Arr_bounded_approximate_vertex<Arrangement>;
  using Bounded_approximate_halfedge = Arr_bounded_approximate_halfedge<Arrangement>;
  using Bounded_render_context = Arr_bounded_render_context<Arrangement>;
  using Triangulator = Arr_bounded_face_triangulator<Arrangement>;

  static constexpr bool Is_on_curved_surface = is_or_derived_from_curved_surf_traits_v<Geom_traits>;

  struct Left_to_right_tag
  {};
  struct Right_to_left_tag
  {};

private:
  class Context : public Bounded_render_context
  {
  public:
    Context(const Bounded_render_context& ctx, Triangulator& triangulator)
        : Bounded_render_context(ctx)
        , m_proj(ctx.m_traits)
        , m_triangulator(triangulator)
        , m_bounded_approx_vertex(ctx)
        , m_bounded_approx_halfedge(ctx) {}
    // Let's not accidentally copy this object.
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;

    void insert(Point pt) {
      if(pt == m_last_pt || !this->contains_x(pt.x())) return;
      pt = Point(pt.x(), std::clamp(pt.y(), this->ymin(), this->ymax()));
      m_last_pt = pt;
      m_triangulator.insert(pt);
    }
    void start_ccb() { m_triangulator.start_ccb(); }
    void end_ccb() { m_triangulator.end_ccb(); }

  private:
    Triangulator& m_triangulator;
    const Arr_projector<Geom_traits> m_proj;

  public:
    Bounded_approximate_vertex m_bounded_approx_vertex;
    Bounded_approximate_halfedge m_bounded_approx_halfedge;
    std::optional<Point> m_last_pt;
  };

private:
  static Arr_parameter_space side_of_fictitious_edge(const Halfedge_const_handle& he) {
    const auto& source = he->source();
    const auto& target = he->target();
    Arr_parameter_space src_x_space = source->parameter_space_in_x();
    Arr_parameter_space src_y_space = source->parameter_space_in_y();
    Arr_parameter_space tgt_x_space = target->parameter_space_in_x();
    Arr_parameter_space tgt_y_space = target->parameter_space_in_y();
    if(src_x_space == tgt_x_space && src_x_space != ARR_INTERIOR) return src_x_space;
    if(src_y_space == tgt_y_space && src_y_space != ARR_INTERIOR) return src_y_space;
    CGAL_assertion(false && "Unexpected parameter space for fictitious edge vertices.");
    return ARR_INTERIOR;
  }

  // Generate dummy segment(directed left to right) for the fictitious edge he.
  static Polyline approximate_fictitious_edge(const Context& ctx, const Halfedge_const_handle& he) {
    Arr_parameter_space side = side_of_fictitious_edge(he);
    // There's no need to handle fictitious edges on left or right boundaries.
    if(side == ARR_LEFT_BOUNDARY || side == ARR_RIGHT_BOUNDARY) return Polyline{};
    if(side == ARR_BOTTOM_BOUNDARY) {
      Point from_pt(ctx.m_last_pt.has_value() ? ctx.m_last_pt->x() : ctx.xmin(), ctx.ymin());
      Point to_pt(ctx.xmax(), ctx.ymin());
      return Polyline{from_pt, to_pt};
    }
    if(side == ARR_TOP_BOUNDARY) {
      Point from_pt(ctx.xmin(), ctx.ymax());
      Point to_pt(ctx.m_last_pt.has_value() ? ctx.m_last_pt->x() : ctx.xmax(), ctx.ymax());
      return Polyline{from_pt, to_pt};
    }
    CGAL_assertion(false && "Unexpected side for a fictitious edge.");
    return Polyline{};
  }

  static void approximate_vertex(Context& ctx, const Vertex_const_handle& vh) {
    if(vh->is_at_open_boundary()) return;
    ctx.m_bounded_approx_vertex(vh);
  }

  template <typename DirectionTag>
  static void approximate_halfedge(Context& ctx, const Halfedge_const_handle& he, DirectionTag dir_tag) {
    const Polyline& polyline =
        he->is_fictitious() ? approximate_fictitious_edge(ctx, he) : ctx.m_bounded_approx_halfedge(he);
    for(const auto& curr_pt : polyline) ctx.insert(curr_pt);
  }

  static void approximate_ccb(Context& ctx, Ccb_halfedge_const_circulator start_circ) {
    // Try to start on a concrete halfedge.
    // For any unbounded face, there can't be more than 4 adjacent fictitious edges.
    for(int i = 0; i < 4 && start_circ->is_fictitious(); ++i) ++start_circ;

    ctx.start_ccb();
    auto circ = start_circ;
    do {
      circ->direction() == ARR_LEFT_TO_RIGHT ? approximate_halfedge(ctx, circ, Left_to_right_tag{})
                                             : approximate_halfedge(ctx, circ, Right_to_left_tag{});
      approximate_vertex(ctx, circ->target());
    } while(++circ != start_circ);
    ctx.end_ccb();
  }

public:
  Arr_bounded_approximate_face(const Bounded_render_context& ctx)
      : m_ctx(ctx) {}

  /*!
   * \brief Approximate an arrangement face with a bunch of triangles.
   *
   * \param fh
   * \return const Triangulated_face&
   */
  const Triangle_soup& operator()(const Face_const_handle& fh) const {
    CGAL_precondition_msg(!fh->is_fictitious(), "Cannot approximate a fictitious face.");

    auto [iter, inserted] = m_ctx.m_cache.faces().try_emplace(fh);
    Triangle_soup& ts = iter->second;
    if(!inserted || m_ctx.is_cancelled()) return ts;
    auto triangulator = Triangulator(m_ctx, fh);
    auto ctx = Context(m_ctx, triangulator);

    if(!Is_on_curved_surface && fh->is_unbounded()) {
      // Skip approximation of the unbounded face in planar arrangements.
      // However, degenerate holes still need to be approximated.
      for(auto inner_ccb = fh->inner_ccbs_begin(); inner_ccb != fh->inner_ccbs_end(); ++inner_ccb) {
        auto circ = *inner_ccb;
        do {
          if(circ->face() != circ->twin()->face()) continue;
          ctx.m_bounded_approx_halfedge(circ);
        } while(++circ != *inner_ccb);
      }
      for(auto vh = fh->isolated_vertices_begin(); vh != fh->isolated_vertices_end(); ++vh)
        ctx.m_bounded_approx_vertex(vh);
      return ts;
    }

    for(auto outer_ccb = fh->outer_ccbs_begin(); outer_ccb != fh->outer_ccbs_end(); ++outer_ccb)
      approximate_ccb(ctx, *outer_ccb);
    for(auto inner_ccb = fh->inner_ccbs_begin(); inner_ccb != fh->inner_ccbs_end(); ++inner_ccb)
      approximate_ccb(ctx, *inner_ccb);
    for(auto iso_vertex = fh->isolated_vertices_begin(); iso_vertex != fh->isolated_vertices_end(); ++iso_vertex)
      approximate_vertex(ctx, iso_vertex);

    return ts = std::move(triangulator);
  }

private:
  const Bounded_render_context& m_ctx;
};

} // namespace draw_aos
} // namespace CGAL

#endif