// Copyright (c) 2025
// Utrecht University (The Netherlands),
// ETH Zurich (Switzerland),
// INRIA Sophia-Antipolis (France),
// Max-Planck-Institute Saarbruecken (Germany),
// and Tel-Aviv University (Israel).  All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL$
// $Id$
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s): Shepard Liu       <shepard0liu@gmail.com>

#ifndef CGAL_DRAW_AOS_ARR_BOUNDED_RENDERER_H
#define CGAL_DRAW_AOS_ARR_BOUNDED_RENDERER_H
#include <optional>

#include <CGAL/Bbox_2.h>
#include <CGAL/Draw_aos/Arr_approximation_cache.h>
#include <CGAL/Draw_aos/Arr_bounded_approximate_face.h>
#include <CGAL/Draw_aos/Arr_render_context.h>
#include <CGAL/Draw_aos/type_utils.h>
#include <utility>
#include <vector>

namespace CGAL {
namespace draw_aos {

/**
 * @brief Render arrangement on surface within a bounding box.
 */
template <typename Arrangement>
class Arr_bounded_renderer
{
  using Geom_traits = typename Arrangement::Geometry_traits_2;
  using Face_const_handle = typename Arrangement::Face_const_handle;
  using Cancellable_render_context = Arr_cancellable_render_context<Arrangement>;
  using Approx_cache = Arr_approximation_cache<Arrangement>;

public:
  using Face_filter = std::vector<bool>;

  Arr_bounded_renderer() = default;

  /*!
   * \brief Render arrangement within the given bounding box.
   * \param ctx The cancellable render context.
   * \param bbox The bounding box to render.
   * \param face_filter Optional filter table to indicate whether a face is inbound. If provided, only faces with true
   * value in the table will be rendered. Faces are indexed in the order of arr.faces_begin() to arr.faces_end().
   *
   * \return std::optional<std::pair<Approx_cache, Face_filter>> Rendering result cache and inbound face table. If
   * the rendering is cancelled, std::nullopt is returned.
   */
  std::optional<std::pair<Approx_cache, Face_filter>>
  render(const Cancellable_render_context& ctx, Bbox_2 bbox, const Face_filter& face_filter = Face_filter()) const {
    Approx_cache cache(bbox);
    cache.vertices().reserve(ctx.m_arr.number_of_vertices());
    cache.halfedges().reserve(ctx.m_arr.number_of_halfedges());
    cache.faces().reserve(ctx.m_arr.number_of_faces());

    Arr_bounded_render_context<Arrangement> derived_ctx(ctx, bbox, cache);
    Arr_bounded_approximate_face<Arrangement> bounded_approx_face(derived_ctx);
    Face_filter is_face_inbound(ctx.m_arr.number_of_faces(), false);
    int idx = 0;
    for(Face_const_handle fh = ctx.m_arr.faces_begin(); fh != ctx.m_arr.faces_end(); ++fh, ++idx) {
      if(ctx.is_cancelled()) return std::nullopt;
      if(!face_filter.empty() && !face_filter[idx]) continue;
      decltype(auto) ts = bounded_approx_face(fh);
      if(!ts.triangles.empty()) is_face_inbound[idx] = true;
    }
    return std::make_pair(std::move(cache), std::move(is_face_inbound));
  }
};

} // namespace draw_aos
} // namespace CGAL

#endif
