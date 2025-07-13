//! \file examples/Arrangement_on_surface_2/unbounded_non_intersecting.cpp
// Constructing an arrangement of unbounded linear objects using the insertion
// function for non-intersecting curves.

#define CGAL_DRAW_AOS_DEBUG
#define CGAL_DRAW_AOS_TRIANGULATOR_DEBUG_FILE_DIR "/Users/shep/codes/aos_2_js_helper"
#include <cassert>

#include "CGAL/draw_arrangement_2.h"
#include "arr_linear.h"
#include "arr_print.h"

int main() {
  Arrangement arr;

  // Insert a line in the (currently single) unbounded face of the arrangement;
  // then, insert a point that lies on the line splitting it into two.
  // X_monotone_curve c1 = Line(Point(-1, 0), Point(1, 0));
  // arr.insert_in_face_interior(c1, arr.unbounded_face());
  // Vertex_handle v = insert_point(arr, Point(0, 0));
  // assert(!v->is_at_open_boundary());

  // // Add two more rays using the specialized insertion functions.
  // arr.insert_from_right_vertex(Ray(Point(0, 0), Point(-1, 1)), v); // c2
  // arr.insert_from_left_vertex(Ray(Point(0, 0), Point(1, 1)), v);   // c3

  // // Insert three more interior-disjoint rays, c4, c5, and c6.
  // insert_non_intersecting_curve(arr, Ray(Point(0, -1), Point(-2, -2)));
  // insert_non_intersecting_curve(arr, Ray(Point(0, -1), Point(2, -2)));
  // insert_non_intersecting_curve(arr, Ray(Point(0, 0), Point(0, 1)));

  int n = 5;
  for(int i = 0; i < n; ++i) {
    Point p1(i * 5, 0);
    Point p2(i * 5, 1);
    CGAL::insert(arr, X_monotone_curve(Line(p1, p2)));
  }
  for(int i = 0; i < n; ++i) {
    Point p1(0, i * 5);
    Point p2(1, i * 5);
    CGAL::insert(arr, X_monotone_curve(Line(p1, p2)));
  }
  // Generate a inner square(2*2) for all cells
  // And an inner triangle for each square
  for(int i = 0; i < n; ++i) {
    for(int j = 0; j < n; ++j) {
      Point p1(i * 5 + 1, j * 5 + 1);
      Point p2(i * 5 + 4, j * 5 + 4);
      CGAL::insert(arr, X_monotone_curve(Segment(p1, Point(p2.x(), p1.y()))));
      CGAL::insert(arr, X_monotone_curve(Segment(Point(p1.x(), p2.y()), p2)));
      CGAL::insert(arr, X_monotone_curve(Segment(p1, Point(p1.x(), p2.y()))));
      CGAL::insert(arr, X_monotone_curve(Segment(Point(p2.x(), p1.y()), p2)));

      // Insert a triangle inside the square
      Point tri_p1(i * 5 + 2, j * 5 + 2);
      Point tri_p2(i * 5 + 3, j * 5 + 2);
      Point tri_p3(i * 5 + 2.5, j * 5 + 3);
      CGAL::insert(arr, X_monotone_curve(Segment(tri_p1, tri_p2)));
      CGAL::insert(arr, X_monotone_curve(Segment(tri_p2, tri_p3)));
      CGAL::insert(arr, X_monotone_curve(Segment(tri_p3, tri_p1)));

      // Connect the triangle to the square
      Point top(i * 5 + 2.5, j * 5 + 4);
      CGAL::insert(arr, X_monotone_curve(Segment(tri_p1, top)));
    }
  }

  CGAL::draw(arr);

  return 0;
}
