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

#ifndef ARR_VIEWER_H
#define ARR_VIEWER_H

#include <OpenGL/gl.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <list>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <type_traits>
#include <utility>

#include <QtGui/QKeyEvent>
#include <QtGui/QMouseEvent>
#include <QtGui/QOpenGLFunctions>
#include <QtOpenGLWidgets/QtOpenGLWidgets>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <vector>

#ifdef CGAL_LINKED_WITH_TBB
#include <tbb/task_group.h>
#include <tbb/mutex.h>
#endif

#include <CGAL/Arr_enums.h>
#include <CGAL/Basic_viewer.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/unordered_flat_map.h>
#include <CGAL/Graphics_scene.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Qt/Basic_viewer.h>
#include <CGAL/Qt/camera.h>
#include <CGAL/Draw_aos/Arr_bounded_renderer.h>
#include <CGAL/Draw_aos/Arr_coordinate_converter.h>
#include <CGAL/Draw_aos/Arr_face_point_generator.h>
#include <CGAL/Draw_aos/Arr_render_context.h>
#include <CGAL/Draw_aos/type_utils.h>

namespace CGAL {
namespace draw_aos {

// Global mutex for all thread-safe cout operations
inline std::mutex& get_cout_mutex() {
  static std::mutex mtx;
  return mtx;
}

// Thread-safe macro
#define COUT(msg)                                                                                                      \
  do {                                                                                                                 \
    std::lock_guard<std::mutex> lock(get_cout_mutex());                                                                \
    std::cout << msg << std::endl;                                                                                     \
  } while(0)

struct Tile_id
{
  Tile_id(int x_, int y_, int level_)
      : x(x_)
      , y(y_)
      , level(level_) {}

  Tile_id parent() const { return Tile_id(x / 2, y / 2, level - 1); }
  bool operator==(const Tile_id& other) const { return x == other.x && y == other.y && level == other.level; }

  const int x, y;
  const int level;
};

std::ostream& operator<<(std::ostream& os, const Tile_id& id) {
  os << id.x << "," << id.y << "@" << id.level;
  return os;
}

struct Tile_id_hash
{
  std::size_t operator()(const Tile_id& id) const {
    std::size_t seed = 0;
    boost::hash_combine(seed, id.x);
    boost::hash_combine(seed, id.y);
    boost::hash_combine(seed, id.level);
    return seed;
  }
};

template <typename Arrangement>
class Tile_ready_event : public QEvent
{
  using Tile_data = Arr_approximation_cache<Arrangement>;

public:
  inline static const QEvent::Type event_type = static_cast<QEvent::Type>(QEvent::registerEventType());

  Tile_ready_event(Tile_id id, std::shared_ptr<Tile_data> obj)
      : QEvent(event_type)
      , m_id(id)
      , m_obj(obj) {}

  const Tile_id& tile_id() const { return m_id; }
  std::shared_ptr<Tile_data> tile_object() const { return m_obj; }

private:
  const Tile_id m_id;
  const std::shared_ptr<Tile_data> m_obj;
};

template <typename Arrangement, typename GSOptions>
class Tile_manager
{
  using Approx_cache = Arr_approximation_cache<Arrangement>;
  using Render_context = Arr_render_context<Arrangement>;
  using Cancellable_render_context = Arr_cancellable_render_context<Arrangement>;
  using Bounded_renderer = Arr_bounded_renderer<Arrangement>;

public:
  using Tile_data = Approx_cache;
  using Tile_face_filter = typename Bounded_renderer::Face_filter;

  struct Tile_object
  {
    Tile_object(Tile_data&& data_, Tile_face_filter&& face_filter_ = Tile_face_filter())
        : data(std::make_shared<Tile_data>(std::move(data_)))
        , face_filter(std::make_shared<Tile_face_filter>(std::move(face_filter_))) {}
    std::shared_ptr<Tile_data> data;
    std::shared_ptr<Tile_face_filter> face_filter;
  };

private:
  using Lru_list = std::list<std::pair<Tile_id, Tile_object>>;
  using Lru_list_iterator = typename Lru_list::iterator;

public:
  Tile_manager(QObject* parent,
               const Arrangement& arr,
               Graphics_scene& scene,
               const GSOptions& options,
               int max_cached_tiles = 100)
      : m_arr(arr)
      , m_capacity(max_cached_tiles)
      , m_gso(options)
      , m_gs(scene)
      , m_parent(parent) {
    CGAL_assertion(m_capacity > 0);
  }

  virtual ~Tile_manager() noexcept {
    tbb::mutex::scoped_lock lock(m_mu);
    for(auto& [_, ctx] : m_pending_tiles) ctx.cancel();
#ifdef CGAL_LINKED_WITH_TBB
    m_task_group.wait();
#endif
  }

private:
  void emit_tile_ready(const Tile_id& id, std::shared_ptr<Tile_data> data) {
    QCoreApplication::postEvent(m_parent, new Tile_ready_event<Arrangement>(id, data));
  }

public:
  /*!
   * \brief Asynchronously render a tile identified by (id) that covers the bounding box (bbox).
   *
   * Posts a Tile_ready_event to the parent widget when the rendering is done.
   * \param ctx
   * \param id
   * \param bbox
   * \return
   */
  void request(Tile_id tid, const Render_context& ctx, Bbox_2 bbox) {
    Cancellable_render_context derived_ctx(ctx);
    std::shared_ptr<Tile_face_filter> parent_filter;
    {
      tbb::mutex::scoped_lock lock(m_mu);
      if(auto it = m_tile_map.find(tid); it != m_tile_map.end()) {
        auto list_it = it->second;
        // Move the used tile to the front of the LRU list.
        m_lru_list.splice(m_lru_list.begin(), m_lru_list, list_it);
        emit_tile_ready(tid, list_it->second.data);
        return;
      }
      auto [_, inserted] = m_pending_tiles.try_emplace(tid, derived_ctx);
      if(!inserted) return;
      // Acquire parent node's face filter to speed up rendering
      if(auto it = m_tile_map.find(tid.parent()); it != m_tile_map.end()) {
        parent_filter = it->second->second.face_filter;
      }
    }

#ifdef CGAL_LINKED_WITH_TBB
    m_task_group.run([this, derived_ctx, tid, bbox, parent_filter]() {
      auto res = Arr_bounded_renderer<Arrangement>().render(derived_ctx, bbox,
                                                            parent_filter ? *parent_filter : std::vector<bool>());
      if(!res.has_value()) return;
      Tile_object obj(std::move(res->first), std::move(res->second));
      {
        tbb::mutex::scoped_lock lock(m_mu);
        m_pending_tiles.erase(tid);
        if(m_lru_list.size() == m_capacity) {
          // Remove the least recently used tile.
          auto list_it = std::prev(m_lru_list.end());
          m_tile_map.erase(list_it->first);
          m_lru_list.pop_back();
        }
        m_lru_list.push_front(std::make_pair(tid, obj));
        m_tile_map[tid] = m_lru_list.begin();
      }
      emit_tile_ready(tid, obj.data);
    });
#endif

    return;
  }

  void cancel(Tile_id tid) {
    tbb::mutex::scoped_lock lock(m_mu);
    if(auto it = m_pending_tiles.find(tid); it != m_pending_tiles.end()) {
      it->second.cancel();
      m_pending_tiles.erase(it);
    }
  }

private:
  const Arrangement& m_arr;
  const GSOptions& m_gso;
  Graphics_scene& m_gs;
  Lru_list m_lru_list;
  int m_capacity;
  QObject* m_parent;
  unordered_flat_map<Tile_id, Lru_list_iterator, Tile_id_hash> m_tile_map;
  unordered_flat_map<Tile_id, Cancellable_render_context, Tile_id_hash> m_pending_tiles;

#ifdef CGAL_LINKED_WITH_TBB
  tbb::mutex m_mu;
  tbb::task_group m_task_group;
#endif
};

/*!
 * \brief Viewport helper functions
 *
 * \tparam Arrangement
 */
template <typename Arrangement, typename = void>
class Arr_viewport_helpers;

// Specialization for planar arrangements
template <typename Arrangement>
class Arr_viewport_helpers<
    Arrangement,
    std::enable_if_t<!is_or_derived_from_curved_surf_traits_v<typename Arrangement::Geometry_traits_2>>>
{
  using Geom_traits = typename Arrangement::Geometry_traits_2;
  using Approx_traits = Arr_approximate_traits<Geom_traits>;
  using Approx_point = typename Approx_traits::Approx_point;
  using Camera = qglviewer::Camera;
  using Point = typename Approx_traits::Point;
  using Local_point = Graphics_scene::Local_point;

private:
  /*!
   * \brief Computes a parameter space bounding box that contains everything in the arrangement with some margin.
   *
   * \note For arrangement induced by unbounded curves, the bounding box only fits all vertices.
   * \return Bbox_2
   */
  Bbox_2 arr_bbox() const {
    const auto& traits = *m_arr.geometry_traits();
    Bbox_2 bbox;
    // Computes a rough bounding box from the vertices.
    for(const auto& vh : m_arr.vertex_handles()) { bbox += traits.approximate_2_object()(vh->point()).bbox(); }
    double approx_error = approximation_error(bbox, 100);
    // Computes a more precise bounding box from the halfedges.
    for(const auto& he : m_arr.halfedge_handles()) {
      traits.approximate_2_object()(
          he->curve(), approx_error,
          boost::make_function_output_iterator([this, &bbox](Approx_point pt) { bbox += pt.bbox(); }));
    }
    // Place margin around the bbox.
    double dx = bbox.x_span() * 0.1;
    double dy = bbox.y_span() * 0.1;
    bbox = Bbox_2(bbox.xmin() - dx, bbox.ymin() - dy, bbox.xmax() + dx, bbox.ymax() + dy);
    // Make sure the bbox is not degenerate.
    if(bbox.x_span() == 0) bbox += Bbox_2(bbox.xmin() - 1, bbox.ymin(), bbox.xmax() + 1, bbox.ymax());
    if(bbox.y_span() == 0) bbox += Bbox_2(bbox.xmin(), bbox.ymin() - 1, bbox.xmax(), bbox.ymax() + 1);
    return bbox;
  }

  /*!
   * \brief Computes parameter space axis aligned bounding box from camera parameters.
   *
   * \param cam
   * \return Bbox_2
   */
  Bbox_2 screen_to_world(const Camera& cam) const {
    QMatrix4x4 mvp;
    cam.getModelViewProjectionMatrix(mvp.data());
    QMatrix4x4 inverse_mvp = mvp.inverted();
    // Define 4 corners of the near plane in NDC (-1 to 1 in x and y)
    std::array<QVector4D, 4> clip_space_corners{QVector4D(-1.0, -1.0, 0.0, 1.0), QVector4D(-1.0, 1.0, 0.0, 1.0),
                                                QVector4D(1.0, -1.0, 0.0, 1.0), QVector4D(1.0, 1.0, 0.0, 1.0)};
    double xmin = std::numeric_limits<double>::max();
    double xmax = std::numeric_limits<double>::lowest();
    double ymin = std::numeric_limits<double>::max();
    double ymax = std::numeric_limits<double>::lowest();
    for(const QVector4D& corner : clip_space_corners) {
      QVector4D world = inverse_mvp * corner;
      if(world.w() != 0.0) world /= world.w();
      double x = world.x();
      double y = world.y();
      xmin = std::min(xmin, x);
      xmax = std::max(xmax, x);
      ymin = std::min(ymin, y);
      ymax = std::max(ymax, y);
    }
    return Bbox_2(xmin, ymin, xmax, ymax);
  }

  double scale(int level) { return level >= 0 ? 1.0 / (1 << level) : (1 << -level); }

protected:
  Arr_viewport_helpers(const Arrangement& arr)
      : m_arr(arr) {}

  void init(Bbox_2 initial_bbox, int viewport_width) {
    CGAL_assertion(!m_inited);
    m_inited = true;
    if(initial_bbox.x_span() == 0 || initial_bbox.y_span() == 0)
      m_bbox_base = arr_bbox();
    else
      m_bbox_base = initial_bbox;
    m_ap_error_base = approximation_error(m_bbox_base, viewport_width);
  }

  /*!
   * \brief Computes a subpixel-level approximation error based on the bounding box and viewport width.
   *
   * \param bbox
   * \param viewport_width width of the viewport in pixels
   * \return double
   */
  double approximation_error(const Bbox_2& bbox, int viewport_width) const { return bbox.x_span() / viewport_width; }

  /*!
   * \brief Fits the camera to bbox.
   *
   * \param bbox
   * \param camera
   */
  void fit_camera(const Bbox_2& bbox, Camera& cam) const {
    using Vec = qglviewer::Vec;
    cam.fitBoundingBox(Vec(bbox.xmin(), bbox.ymin(), 0.0), Vec(bbox.xmax(), bbox.ymax(), 0.0));
  }

  std::pair<Bbox_2, double> tile_meta(Tile_id tid) {
    double tile_width = m_bbox_base.x_span() * scale(tid.level);
    double tile_height = m_bbox_base.y_span() * scale(tid.level);
    Bbox_2 bbox(m_bbox_base.xmin() + tid.x * tile_width, m_bbox_base.ymin() + tid.y * tile_height,
                m_bbox_base.xmin() + (tid.x + 1) * tile_width, m_bbox_base.ymin() + (tid.y + 1) * tile_height);
    double approx_error = scale(tid.level) * m_ap_error_base;
    return std::make_pair(bbox, approx_error);
  }

  template <typename OutputIterator>
  void get_tiles_from_camera(const Camera& cam, int viewport_width, OutputIterator out_it) {
    Bbox_2 bbox = screen_to_world(cam);
    double ap_error = approximation_error(bbox, viewport_width);
    int level = std::ceil(std::log2(m_ap_error_base / ap_error));
    double tile_width = m_bbox_base.x_span() * scale(level);
    double tile_height = m_bbox_base.y_span() * scale(level);
    int x_start = std::floor((bbox.xmin() - m_bbox_base.xmin()) / tile_width);
    int x_end = std::ceil((bbox.xmax() - m_bbox_base.xmin()) / tile_width);
    int y_start = std::floor((bbox.ymin() - m_bbox_base.ymin()) / tile_height);
    int y_end = std::ceil((bbox.ymax() - m_bbox_base.ymin()) / tile_height);
    for(int x = x_start; x < x_end; ++x)
      for(int y = y_start; y < y_end; ++y) *out_it++ = Tile_id(x, y, level);
  }

  /*!
   * \brief Converts a parameter space point to a local point of the buffer object.
   *
   * \param pt
   * \return Local_point
   */
  Local_point to_local_point(Point pt) const { return Local_point(pt.x(), pt.y(), 0.0); }

  const Bbox_2& base_bbox() const { return m_bbox_base; }

private:
  const Arrangement& m_arr;
  bool m_inited{false};
  Bbox_2 m_bbox_base;
  double m_ap_error_base;
};

// Spherical arrangement specialization
template <typename Arrangement>
class Arr_viewport_helpers<Arrangement,
                           std::enable_if_t<is_or_derived_from_agas_v<typename Arrangement::Geometry_traits_2>>>
{
  using Geom_traits = typename Arrangement::Geometry_traits_2;
  using Approx_traits = Arr_approximate_traits<Geom_traits>;
  using Approx_point = typename Approx_traits::Approx_point;
  using Camera = qglviewer::Camera;
  using Point = typename Approx_traits::Point;
  using Local_point = Graphics_scene::Local_point;

protected:
  Arr_viewport_helpers(const Arrangement& arr)
      : m_arr(arr) {}

  Bbox_2 arr_bbox() const { return Bbox_2(0, 0, 2 * CGAL_PI, CGAL_PI); }

  Bbox_2 screen_to_world(const Camera& cam) const { return Bbox_2(0, 0, 2 * CGAL_PI, CGAL_PI); }

  void fit_camera(const Bbox_2&, Camera& cam) {
    using Vec = qglviewer::Vec;
    cam.setSceneCenter(Vec(0, 0, 0));
    cam.fitSphere(Vec(0, 0, 0), 1.1); // slightly larger than the unit sphere
  }

  double approximation_error(const Bbox_2& bbox, int viewport_width) const {
    // If crossing hemisphere
    if(bbox.x_span() >= CGAL_PI) return 1.0 / viewport_width;
    // Otherwise we evalute the error bound with respect to the longest longitude arc
    double theta =
        std::abs(bbox.ymin() - CGAL_PI / 2.0) < std::abs(bbox.ymax() - CGAL_PI / 2.0) ? bbox.ymin() : bbox.ymax();
    return bbox.x_span() * std::sin(theta) / viewport_width;
  }

  Graphics_scene::Local_point to_local_point(Point pt) const {
    auto approx_pt = Arr_coordinate_converter(*m_arr.geometry_traits()).to_cartesian(pt);
    return Graphics_scene::Local_point(approx_pt.dx(), approx_pt.dy(), approx_pt.dz());
  }

private:
  const Arrangement& m_arr;
};

/*! Viewer for visualizing arrangements on surface.
 *
 * \tparam Arrangement
 * \tparam GSOptions
 */
template <typename Arrangement, typename GSOptions>
class Arr_viewer : public Qt::Basic_viewer, Arr_viewport_helpers<Arrangement>
{
  using Basic_viewer = Qt::Basic_viewer;
  using Helpers = Arr_viewport_helpers<Arrangement>;
  using Vertex_const_handle = typename Arrangement::Vertex_const_handle;
  using Halfedge_const_handle = typename Arrangement::Halfedge_const_handle;
  using Face_const_handle = typename Arrangement::Face_const_handle;
  using Geom_traits = typename Arrangement::Geometry_traits_2;
  using Approx_traits = Arr_approximate_traits<Geom_traits>;
  using Approx_point = typename Approx_traits::Approx_point;
  using Point = typename Approx_traits::Point;
  using Point_generator = Arr_face_point_generator<Arrangement>;
  using Faces_point_map = typename Point_generator::Face_points_map;
  using Tile_manager_ = Tile_manager<Arrangement, GSOptions>;
  using Tile_data = typename Tile_manager_::Tile_data;
  using Tile_map = unordered_flat_map<Tile_id, std::shared_ptr<Tile_data>, Tile_id_hash>;
  constexpr static bool Is_on_curved_surface = is_or_derived_from_curved_surf_traits_v<Geom_traits>;

  class Scene_ready_event : public QEvent
  {
  public:
    inline static const QEvent::Type event_type = static_cast<QEvent::Type>(QEvent::registerEventType());
    Scene_ready_event()
        : QEvent(event_type) {}
  };

private:
  static bool contains(const Bbox_2& bbox, const Point& pt) {
    return bbox.xmin() <= pt.x() && pt.x() <= bbox.xmax() && bbox.ymin() <= pt.y() && pt.y() <= bbox.ymax();
  }

  int viewport_width() const {
    std::array<GLint, 4> viewport;
    this->camera_->getViewport(viewport.data());
    return viewport[2];
  }

  /*!
   * \brief Render a single tile to the scene
   *
   * \param tile
   */
  void render_tile(const Tile_data& tile, Graphics_scene& scene) {
    // add faces
    for(const auto& [fh, tf] : tile.faces()) {
      if(!m_gso.draw_face(m_arr, fh)) continue;
      bool colored_face = m_gso.colored_face(m_arr, fh);
      auto color = colored_face ? m_gso.face_color(m_arr, fh) : CGAL::IO::Color();
      for(const auto& tri : tf.triangles) {
        if(colored_face)
          scene.face_begin(color);
        else
          scene.face_begin();
        for(const auto i : tri) scene.add_point_in_face(this->to_local_point(tf.points[i]));
        scene.face_end();
      }
    }
    // add edges
    for(const auto& [he, polyline] : tile.halfedges()) {
      if(he->direction() == ARR_RIGHT_TO_LEFT || !m_gso.draw_edge(m_arr, he) || polyline.size() < 2) continue;
      bool colored_edge = m_gso.colored_edge(m_arr, he);
      auto color = colored_edge ? m_gso.edge_color(m_arr, he) : CGAL::IO::Color();
      // skip first two if starts with a sep point.
      int start_idx = Approx_traits::is_null(polyline.front()) ? 2 : 0;
      // skip last two if ends with a sep point.
      int end_idx = Approx_traits::is_null(polyline.back()) ? polyline.size() - 2 : polyline.size();
      for(int i = start_idx; i < end_idx - 1; ++i) {
        const auto& src = polyline[i];
        const auto& tgt = polyline[i + 1];
        if(Approx_traits::is_null(src) || Approx_traits::is_null(tgt)) continue;
        if(!contains(tile.bbox(), src) || !contains(tile.bbox(), tgt)) continue;
        if(colored_edge)
          scene.add_segment(this->to_local_point(src), this->to_local_point(tgt), color);
        else
          scene.add_segment(this->to_local_point(src), this->to_local_point(tgt));
      }
    }
    // add vertices
    for(const auto& [vh, pt] : tile.vertices()) {
      if(!m_gso.draw_vertex(m_arr, vh) || !contains(tile.bbox(), pt)) continue;
      if(m_gso.colored_vertex(m_arr, vh))
        scene.add_point(this->to_local_point(pt), m_gso.vertex_color(m_arr, vh));
      else
        scene.add_point(this->to_local_point(pt));
    }
  }

  void request_tiles() {
    Tile_map new_tiles;
    this->get_tiles_from_camera(
        *this->camera_, viewport_width(),
        boost::make_function_output_iterator([&new_tiles](Tile_id tid) { new_tiles.emplace(tid, nullptr); }));

    std::vector<Tile_id> to_cancel;
    to_cancel.reserve(new_tiles.size());
    std::vector<Tile_id> to_request;
    to_request.reserve(new_tiles.size());
    {
      std::lock_guard<std::mutex> lock(m_data_mu);
      for(auto& [tid, tile] : m_cur_tiles) {
        if(auto it = new_tiles.find(tid); it != new_tiles.end()) {
          // reuse existing tile if still needed
          it->second = tile;
          continue;
        }
        // tile is no longer needed, we cancel its rendering if it is still in progress.
        if(tile == nullptr) to_cancel.push_back(tid);
      }
      for(const auto& [tid, tile] : new_tiles) {
        if(tile != nullptr) continue;
        to_request.push_back(tid);
      }
      m_cur_tiles = std::move(new_tiles);
    }
    for(const auto& tid : to_cancel) m_tm.cancel(tid);
    for(const auto& tid : to_request) {
      auto [bbox, approx_error] = this->tile_meta(tid);
      Arr_render_context<Arrangement> ctx(m_arr, approx_error, m_face_points);
      this->m_tm.request(tid, ctx, bbox);
    }
  }

  bool is_camera_changed() {
    std::array<GLdouble, 16> mvp;
    this->camera_->getModelViewProjectionMatrix(mvp.data());
    if(mvp != m_last_mvp) {
      m_last_mvp = mvp;
      return true;
    }
    return false;
  }

  void handle_tile_ready(const Tile_ready_event<Arrangement>& ev) {
    std::lock_guard<std::mutex> lock(m_data_mu);
    if(auto it = m_cur_tiles.find(ev.tile_id()); it != m_cur_tiles.end()) {
      it->second = ev.tile_object();
      m_data_outdated = true;
    }
  }

  void handle_scene_ready() { redraw(); }

#ifdef CGAL_DRAW_AOS_DEBUG
  void debug_draw_tile_info(const Tile_id& tid, Graphics_scene& scene) {
    if constexpr(!Is_on_curved_surface) {
      auto [bbox, _] = this->tile_meta(tid);
      // Show tile id at bbox center
      auto center = Approx_point((bbox.xmin() + bbox.xmax()) / 2.0, (bbox.ymin() + bbox.ymax()) / 2.0);
      scene.add_text(this->to_local_point(center),
                     std::to_string(tid.x) + "," + std::to_string(tid.y) + "@" + std::to_string(tid.level));
      // Draw the bounding box
      Local_point bl(bbox.xmin(), bbox.ymin(), 0.0);
      Local_point br(bbox.xmax(), bbox.ymin(), 0.0);
      Local_point tl(bbox.xmin(), bbox.ymax(), 0.0);
      Local_point tr(bbox.xmax(), bbox.ymax(), 0.0);
      CGAL::Color box_color(255, 0, 0);
      scene.add_segment(bl, br, box_color);
      scene.add_segment(br, tr, box_color);
      scene.add_segment(tr, tl, box_color);
      scene.add_segment(tl, bl, box_color);
    }
  }
#endif

public:
  Arr_viewer(QWidget* parent, const Arrangement& arr, const GSOptions& gso, const char* title, Bbox_2 initial_bbox)
      : Basic_viewer(parent, m_gs, title)
      , Helpers(arr)
      , m_gso(gso)
      , m_arr(arr)
      , m_coords(*arr.geometry_traits())
      , m_tm(this, arr, m_gs, gso) {
    // Disable lighting for 2d scenarios
    if constexpr(!Is_on_curved_surface) {
      this->m_diffuse_color = QVector4D(0.0f, 0.0f, 0.0f, 0.0f);
      this->m_ambient_color = QVector4D(1.0f, 1.0f, 1.0f, 1.0f);
    }
    // QSurfaceFormat format;
    // format.setSamples(4);
    // this->setFormat(format);

    Helpers::init(initial_bbox, viewport_width());
    m_face_points = Point_generator()(arr, 0.1);

    // Start a new thread for preparing the graphics scene whenever the tiles are outdated.
    m_render_thread = std::thread([this]() mutable {
      std::vector<std::pair<Tile_id, std::shared_ptr<Tile_data>>> tiles;
      while(!m_destructed) {
        if(!m_data_outdated) continue;
        {
          std::lock_guard<std::mutex> lock(m_data_mu);
          m_data_outdated = false;
          // Collect render-ready tiles
          for(const auto& [tid, tile] : m_cur_tiles) {
            if(tile != nullptr) tiles.emplace_back(tid, tile);
          }
        }
        if(tiles.empty()) continue;
        // Atomically produces an up-to-date scene buffer
        {
          std::lock_guard<std::mutex> lock(m_buf_mu);
          m_scene_outdated = true;
          m_buf_gs.clear();
          for(const auto& [tid, tile] : tiles) {
            render_tile(*tile, m_buf_gs);
#ifdef CGAL_DRAW_AOS_DEBUG
            debug_draw_tile_info(tid, m_buf_gs);
#endif
          }
        }
        QCoreApplication::postEvent(this, new Scene_ready_event());
        tiles.clear();
      }
    });
  }

  ~Arr_viewer() {
    m_destructed = true;
    m_render_thread.join();
  }

  virtual void draw() override {
    // Recompute and request visible tiles if the camera parameters changed.
    if(is_camera_changed()) request_tiles();
    Basic_viewer::draw();
  }

  virtual void redraw() override {
    if(!m_initialized) {
      // The initial render must be done with original camera parameters or the width of edges gets exaggerated.
      // So we fit the camera after initial render.
      this->fit_camera(this->base_bbox(), *this->camera_);
      request_tiles();
      m_initialized = true;
    }
    if(!m_scene_outdated) return; // Precheck is ok because we only have one consumer thread.
    if(auto lock = std::unique_lock(m_buf_mu, std::try_to_lock); lock.owns_lock()) {
      // Atomically consumes an up-to-date scene buffer
      std::swap(m_gs, m_buf_gs);
      m_scene_outdated = false;
    } else {
      // Skip this redraw if the scene is being updated
      return draw();
    }
    Basic_viewer::redraw();
  }

protected:
  virtual void customEvent(QEvent* event) override {
    if(event->type() == Tile_ready_event<Arrangement>::event_type) {
      auto* tre = static_cast<Tile_ready_event<Arrangement>*>(event);
      handle_tile_ready(*tre);
      event->accept();
    } else if(event->type() == Scene_ready_event::event_type) {
      handle_scene_ready();
      event->accept();
    } else {
      Basic_viewer::customEvent(event);
    }
  }

  virtual void initializeGL() override {
    Basic_viewer::initializeGL();
    glEnable(GL_MULTISAMPLE);
  }

private:
  Graphics_scene m_gs;
  GSOptions m_gso;
  const Arrangement& m_arr;
  bool m_initialized{false};
  bool m_destructed{false};
  const Arr_coordinate_converter<Geom_traits> m_coords;
  Tile_manager_ m_tm;
  Tile_map m_cur_tiles;
  Faces_point_map m_face_points;
  std::array<GLdouble, 16> m_last_mvp;
  // Shared data for rendering
  std::atomic<bool> m_data_outdated{false};  // Whether the scene data needs to be recomputed from tiles.
  std::atomic<bool> m_scene_outdated{false}; // Whether the scene needs to be redrawn.
  Graphics_scene m_buf_gs;
  std::mutex m_buf_mu;
  std::mutex m_data_mu;
  std::thread m_render_thread;
};

} // namespace draw_aos
} // namespace CGAL

#endif
