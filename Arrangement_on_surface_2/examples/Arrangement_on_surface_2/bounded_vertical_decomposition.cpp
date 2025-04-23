//! \file examples/Arrangement_on_surface_2/vertical_decomposition.cpp
// Performing vertical decomposition of an arrangement.

#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_on_surface_2.h>
#include <CGAL/Arrangement_with_history_2.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/Object.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/draw_arrangement_2.h>
#include <CGAL/intersections.h>
#include <iostream>

#include <CGAL/Arr_vertical_decomposition_2.h>
#include <CGAl/draw_polygon_2.h>
#include <QGraphicsPolygonItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QRandomGenerator>
#include <iterator>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;

using Traits = CGAL::Arr_segment_traits_2<Kernel>;

using Point = Traits::Point_2;

using Segment = Traits::X_monotone_curve_2;

using Arrangement = CGAL::Arrangement_with_history_2<Traits>;
using Vertex_handle = Arrangement::Vertex_handle;
using Halfedge_handle = Arrangement::Halfedge_handle;
using Face_handle = Arrangement::Face_handle;
using Vertex_const_handle = Arrangement::Vertex_const_handle;
using Halfedge_const_handle = Arrangement::Halfedge_const_handle;
using Face_const_handle = Arrangement::Face_const_handle;
using Object_pair = std::pair<CGAL::Object, CGAL::Object>;
using Vert_decomp_entry = std::pair<Vertex_const_handle, Object_pair>;

QRandomGenerator randGen(1);

int visualizePolygons(std::vector<QPolygonF>& polygons) {
  int argc = 0;
  char* argv[] = {nullptr};
  QApplication app(argc, argv);
  QGraphicsScene* scene = new QGraphicsScene();

  scene->setBackgroundBrush(QBrush(Qt::white, Qt::SolidPattern));

  QRandomGenerator gen = randGen;
  for(auto& polygon : polygons) {
    // generate a random color
    int r = gen.bounded(256);
    int g = gen.bounded(256);
    int b = gen.bounded(256);
    QPen pen(Qt::black);
    pen.setJoinStyle(Qt::BevelJoin);
    pen.setWidthF(0.01);
    QBrush brush(QColor(r, g, b));
    scene->addPolygon(polygon, pen, brush);
  }

  QGraphicsView view;
  view.setRenderHint(QPainter::Antialiasing);
  view.setRenderHint(QPainter::TextAntialiasing);
  view.setRenderHint(QPainter::SmoothPixmapTransform);
  view.setScene(scene);
  view.setTransform(QTransform::fromScale(1, -1));
  // set the scene rect to the bounding rect of the polygons
  QRectF boundingRect;
  for(const auto& polygon : polygons) {
    boundingRect = boundingRect.united(polygon.boundingRect());
  }
  view.setWindowTitle("Arrangement on surface 2");
  view.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
  view.setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
  view.setMinimumSize(800, 600);
  scene->setSceneRect(boundingRect);
  view.fitInView(boundingRect, Qt::KeepAspectRatio);

  view.show();
  return app.exec();
}

bool isConnected(Vertex_const_handle from, Vertex_const_handle to) {
  if(from->is_isolated()) {
    return false;
  }
  auto ihf = from->incident_halfedges()->ccb();
  auto start = ihf;
  do {
    if(ihf->twin()->source() == to) {
      return true;
    }
    ihf = ihf->next();
  } while(ihf != start);
  return false;
}

void print_face(Halfedge_const_handle h, const std::string& name = "") {
  Face_const_handle f = h->face();
  bool isDegenerate = h->twin()->face() == f;

  std::cout << "-------------------------" << std::endl;
  std::cout << "[" << name << "]" << std::endl;
  std::cout << "Degenerate: " << (isDegenerate ? "true" : "false") << std::endl;
  if(isDegenerate == false) {
    std::cout << "Face has " << f->number_of_outer_ccbs() << " outer ccb(s) and " << f->number_of_inner_ccbs()
              << " inner ccb(s)." << std::endl;
    std::cout << "Face is " << (f->is_unbounded() ? "unbounded" : "bounded") << std::endl;
    std::cout << "Face's outer ccb vertices: ";
  }

  Halfedge_const_handle hf = isDegenerate ? h : f->outer_ccb()->ccb();
  auto start = hf;

  do {
    std::cout << "(" << hf->source()->point().x() << ", " << hf->source()->point().y() << ") ";
    hf = hf->next();
  } while(hf != start);
  std::cout << std::endl;

  if(!isDegenerate) {
    for(auto inner_ccb = f->inner_ccbs_begin(); inner_ccb != f->inner_ccbs_end(); inner_ccb++) {
      std::cout << "Face's inner ccb vertices: ";
      Halfedge_const_handle ihf = (*inner_ccb)->ccb();
      auto start = ihf;
      do {
        std::cout << "(" << ihf->source()->point().x() << ", " << ihf->source()->point().y() << ") ";
        ihf = ihf->next();
      } while(ihf != start);
      std::cout << std::endl;
    }
  }
  std::cout << "-------------------------" << std::endl;
}

bool construct_path(CGAL::Object& obj, Arrangement& arr, Vertex_handle& vh, Segment& seg) {
  Halfedge_const_handle feature_he_const;
  Vertex_const_handle feature_vertex_const;
  auto& tt = *arr.traits();

  if(CGAL::assign(feature_he_const, obj)) {
    Halfedge_handle feature_he(feature_he_const.ptr());
    bool isEdgeOfThisFace = isConnected(vh, feature_he->source());
    if(!isEdgeOfThisFace) {
      Traits::Intersect_2 intersect_2 = tt.intersect_2_object();
      Traits::X_monotone_curve_2 ray;
      ray = Segment(vh->point(), {vh->point().x(), feature_he->curve().bbox().ymax()});

      using Point_intersect_type = std::pair<Traits::Point_2, Traits::Multiplicity>;
      using Intersect_type = std::variant<Point_intersect_type, Traits::X_monotone_curve_2>;
      std::vector<Intersect_type> points;
      intersect_2(ray, feature_he->curve(), std::back_inserter(points));

      Traits::Point_2 intersection_point = std::get<Point_intersect_type>(points[0]).first;
      seg = Segment(vh->point(), intersection_point);
      return true;
    }
  } else if(CGAL::assign(feature_vertex_const, obj)) {
    Vertex_handle feature_vertex(feature_vertex_const.ptr());
    bool isVertexOfThisFace = isConnected(vh, feature_vertex);
    if(!isVertexOfThisFace) {
      seg = Segment(vh->point(), feature_vertex->point());
      return true;
    }
  }

  return false;
}

void insert_polygon(Arrangement& arr, const std::vector<Point>& polygon) {
  std::vector<Segment> segments;
  for(size_t i = 0; i < polygon.size(); ++i) {
    segments.emplace_back(polygon[i], polygon[(i + 1) % polygon.size()]);
  }
  CGAL::insert(arr, segments.begin(), segments.end());
}

int main(int argc, char* argv[]) {

  Arrangement arr;

  std::vector<Point> outerPolygon{{-3, 0}, {-1, 3}, {3, 2}, {2, -3}, {1, -5}, {0, -5}, {-1, -1}};
  std::vector<Point> innerPoly1{{-2, 1}, {-1, 2}, {-1, -1}};
  std::vector<Point> innerPoly2{{0, 0}, {2, 1}, {0, 2}, {1, 1}};
  std::vector<Point> innerPoly3{{0, -1}, {0, -2}, {1, -2}, {1, -1}};
  std::vector<Point> innerPoly4{{-0.25, -3}, {1, -3}, {0, -4}};
  std::vector<Point> innerPoly5{{0.5, -1.25}, {0.25, -1.75}, {0.75, -1.75}};
  std::vector<std::vector<Point>> innerPolygons{innerPoly1, innerPoly2, innerPoly3, innerPoly4, innerPoly5};
  // std::vector<std::vector<Point>> innerPolygons{innerPoly4};
  insert_polygon(arr, outerPolygon);
  for(const auto& innerPoly : innerPolygons) {
    insert_polygon(arr, innerPoly);
  }
  CGAL::insert(arr, Segment(Point(-0.5, 0), Point(2, 0)));
  CGAL::insert(arr, Segment(Point(-0.5, 0.5), Point(-0.25, 1)));

  std::unordered_map<Vertex_const_handle, Object_pair> decomp_results;

  CGAL::decompose(arr, std::inserter(decomp_results, decomp_results.end()));

  arr.edges_begin()->direction();

  std::cout << "The arrangement has " << arr.number_of_faces() << " faces." << std::endl;
  std::cout << "decompose_results.size() = " << decomp_results.size() << std::endl;

  for(auto& f : arr.face_handles()) {
    if(f->is_unbounded()) {
      continue;
    }

    std::cout << "face has " << f->number_of_outer_ccbs() << " outer ccb(s) and " << f->number_of_inner_ccbs()
              << " inner ccb(s)." << std::endl;

    std::vector<Segment> segs_to_insert;

    for(auto inner_ccb = f->inner_ccbs_begin(); inner_ccb != f->inner_ccbs_end(); inner_ccb++) {
      auto he = (*inner_ccb)->ccb();
      auto start = he;
      do {
        auto it = decomp_results.find(he->source());
        if(it == decomp_results.end()) {
          throw std::runtime_error("??????");
        }
        Vertex_handle vh = he->source();
        Face_handle fh = he->twin()->face();
        auto& entry = it->second;
        auto& above = entry.second;

        Segment seg;
        if(construct_path(above, arr, vh, seg)) {
          segs_to_insert.push_back(seg);
          break;
        }

      } while(++he != start);
    }

    CGAL::insert(arr, segs_to_insert.begin(), segs_to_insert.end());
  }

  std::cout << "After decompose and insertions, the arrangement has " << arr.number_of_faces() << " faces."
            << std::endl;

  std::vector<QPolygonF> decomposed_polygons;
  for(auto& f : arr.face_handles()) {
    if(f->is_unbounded()) {
      continue;
    }
    CGAL::Polygon_2<Kernel> polygon;
    QPolygonF qpolygon;

    auto ccb = f->outer_ccb();
    auto start = ccb;
    std::cout << "generating polygon" << std::endl;
    do {
      auto vertex = ccb->source();
      polygon.push_back(vertex->point());
      qpolygon.push_back(QPointF(CGAL::to_double(vertex->point().x()), CGAL::to_double(vertex->point().y())));
    } while(++ccb != start);
    decomposed_polygons.push_back(qpolygon);

    visualizePolygons(decomposed_polygons);
  }
  std::cout << "Decomposed polygons: " << decomposed_polygons.size() << std::endl;
}
