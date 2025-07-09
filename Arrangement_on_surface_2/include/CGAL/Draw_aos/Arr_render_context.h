#ifndef CGAL_DRAW_AOS_ARR_RENDER_CONTEXT_H
#define CGAL_DRAW_AOS_ARR_RENDER_CONTEXT_H
#include "CGAL/Arr_point_location_result.h"
#include "CGAL/Arr_trapezoid_ric_point_location.h"
#include "CGAL/Bbox_2.h"
#include "CGAL/Draw_aos/Arr_approximate_point_2.h"
#include "CGAL/Draw_aos/Arr_approximation_cache.h"
#include "CGAL/Draw_aos/Arr_construct_curve_end.h"
#include "CGAL/Draw_aos/Arr_construct_segments.h"
#include "CGAL/Draw_aos/Arr_portals.h"
#include "CGAL/Draw_aos/type_utils.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Draw_aos/helpers.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>

namespace CGAL {
namespace draw_aos {

class Arr_cancellable_context_mixin
{
  using Clock = std::chrono::steady_clock;
  using Duration = Clock::duration;
  using Time_point = std::chrono::time_point<Clock, Duration>;

protected:
  Arr_cancellable_context_mixin()
      : m_start_time(Clock::now())
      , m_done(std::make_shared<std::atomic<bool>>(false)) {}

public:
  Time_point start_time() const { return m_start_time; }

  Time_point end_time() const { return m_end_time; }

  Duration elapsed_time() const { return Clock::now() - m_start_time; }

  bool is_cancelled() const { return m_done->load(); }

  void cancel() {
    m_done->store(true, std::memory_order_relaxed);
    m_end_time = Clock::now();
  }

private:
  Time_point m_start_time, m_end_time;
  std::shared_ptr<std::atomic<bool>> m_done;
};

class Arr_bounds_context_mixin
{
  using Approx_point = Arr_approximation_geometry_traits::Approx_point;
  using Point_2 = Type_traits<Geom_traits>::Point_2;
  using FT = Type_traits<Geom_traits>::FT;

protected:
  Arr_bounds_context_mixin(const Bbox_2& bbox)
      : m_bbox(bbox) {}

public:
  double xmin() const { return m_bbox.xmin(); }
  double xmax() const { return m_bbox.xmax(); }
  double ymin() const { return m_bbox.ymin(); }
  double ymax() const { return m_bbox.ymax(); }

  const Bbox_2& bbox() const { return m_bbox; }

  bool strictly_contains_x(double x) const { return xmin() < x && x <= xmax(); }
  bool strictly_contains_x(FT x) const { return FT(xmin()) < x && x <= FT(xmax()); }

  bool strictly_contains_y(double y) const { return ymin() < y && y <= ymax(); }
  bool strictly_contains_y(FT y) const { return FT(ymin()) < y && y <= FT(ymax()); }

  bool strictly_contains(const Point_2& pt) const { return strictly_contains_x(pt.x()) && strictly_contains_y(pt.y()); }
  bool strictly_contains(const Approx_point& pt) const {
    return strictly_contains_x(pt.x()) && strictly_contains_y(pt.y());
  }

  bool contains_x(double x) const { return xmin() <= x && x <= xmax(); }
  bool contains_x(FT x) const { return FT(xmin()) <= x && x <= FT(xmax()); }

  bool contains_y(double y) const { return ymin() <= y && y <= ymax(); }
  bool contains_y(FT y) const { return FT(ymin()) <= y && y <= FT(ymax()); }

  bool contains(const Approx_point& pt) const { return contains_x(pt.x()) && contains_y(pt.y()); }
  bool contains(const Point_2& pt) const { return contains_x(pt.x()) && contains_y(pt.y()); }

  bool is_on_boundary(const Approx_point& pt) const {
    return (pt.x() == xmin() || pt.x() == xmax()) && contains_y(pt.y()) ||
           (pt.y() == ymin() || pt.y() == ymax()) && contains_x(pt.x());
  }
  bool is_on_boundary(const Point_2& pt) const {
    return (pt.x() == FT(xmin()) || pt.x() == FT(xmax())) && contains_y(pt.y()) ||
           (pt.y() == FT(ymin()) || pt.y() == FT(ymax())) && contains_x(pt.x());
  }

private:
  const Bbox_2 m_bbox;
};

class Arr_geom_traits_context_mixin
{
public:
  Arr_geom_traits_context_mixin(const Geom_traits& _traits)
      : traits(_traits)
      , cst_curve_end(_traits)
      , cst_horizontal_segment(_traits)
      , cst_vertical_segment(_traits)
      , intersect_2(_traits.intersect_2_object())
      , compare_xy_2(_traits.compare_xy_2_object())
      , is_vertical_2(_traits.is_vertical_2_object())
      , approx_pt(_traits) {}

  const Geom_traits& traits;
  const Arr_construct_curve_end<Geom_traits> cst_curve_end;
  const Arr_construct_vertical_segment<Geom_traits> cst_vertical_segment;
  const Arr_construct_horizontal_segment<Geom_traits> cst_horizontal_segment;
  const Type_traits<Geom_traits>::Intersect_2 intersect_2;
  const Type_traits<Geom_traits>::Compare_xy_2 compare_xy_2;
  const Type_traits<Geom_traits>::Is_vertical_2 is_vertical_2;
  const Arr_approximate_point_2<Geom_traits> approx_pt;
};

class Arr_render_context : public Arr_cancellable_context_mixin, public Arr_geom_traits_context_mixin
{
  using Point_location = Arr_trapezoid_ric_point_location<Arrangement>;
  using Feature_portals_map = Arr_portals::Feature_portals_map;

public:
  Arr_render_context(const Arrangement& arr,
                     const Point_location& pl,
                     const Feature_portals_map& feature_portals,
                     double approx_error)
      : Arr_cancellable_context_mixin()
      , Arr_geom_traits_context_mixin(*arr.traits())
      , arr(arr)
      , point_location(pl)
      , feature_portals(feature_portals)
      , counter(std::make_shared<std::size_t>(0)) // TODO: remove this after debugging
      , approx_error(approx_error) {}

public:
  const double approx_error;
  const Arrangement& arr;
  std::shared_ptr<std::size_t> counter; // TODO: remove this after debugging
  const Point_location& point_location;
  const Feature_portals_map& feature_portals;
};

class Arr_bounded_render_context : public Arr_render_context, public Arr_bounds_context_mixin
{
  using Approx_point = Arr_approximation_geometry_traits::Approx_point;
  using Point_2 = Type_traits<Geom_traits>::Point_2;

  constexpr static double ep_base = std::numeric_limits<double>::epsilon();

public:
  Arr_bounded_render_context(const Arr_render_context& ctx, const Bbox_2& bbox, Arr_approximation_cache& cache)
      : Arr_render_context(ctx)
      , ep_xmin(std::max(std::abs(ep_base * bbox.xmin()), ep_base))
      , ep_xmax(std::max(std::abs(ep_base * bbox.xmax()), ep_base))
      , ep_ymin(std::max(std::abs(ep_base * bbox.ymin()), ep_base))
      , ep_ymax(std::max(std::abs(ep_base * bbox.ymax()), ep_base))
      , Arr_bounds_context_mixin(bbox)
      , cache(cache) {

    // TODO: remove this after debugging
    std::ofstream ofs_index("/Users/shep/codes/aos_2_js_helper/shapes.txt", std::ios::out | std::ios::trunc);
  }

  Approx_point approx_pt_on_boundary(const Point_2& pt) const {
    double x = this->approx_pt(pt, 0);
    double y = this->approx_pt(pt, 1);

    if(std::abs(x - xmin()) < ep_xmin) {
      x = xmin();
    } else if(std::abs(x - xmax()) < ep_xmax) {
      x = xmax();
    } else if(std::abs(y - ymin()) < ep_ymin) {
      y = ymin();
    } else if(std::abs(y - ymax()) < ep_ymax) {
      y = ymax();
    } else {
      std::cerr << "Failed to match point to the boundary: " << pt << std::endl;
      std::cerr << "Bbox: " << bbox() << std::endl;
      // We shall not call this function if not approximated from a boundary point.
      CGAL_assertion(false && "Failed to match point to the boundary");
    }

    return Approx_point(x, y);
  }

public:
  Arr_approximation_cache& cache;

private:
  // floating point epsilon at boundary coordinates
  double ep_xmin, ep_xmax, ep_ymin, ep_ymax;
};

template <typename Context>
class Arr_context_delegator
{
public:
  Arr_context_delegator(const Context& ctx)
      : m_ctx(ctx) {}

  const Context* operator->() const { return &m_ctx; }
  const Context& operator*() const { return m_ctx; }

  // implicit conversion to the context type
  operator const Context&() const { return m_ctx; }

private:
  const Context& m_ctx;
};

} // namespace draw_aos
} // namespace CGAL
#endif // CGAL_DRAW_AOS_ARR_RENDER_CONTEXT_H