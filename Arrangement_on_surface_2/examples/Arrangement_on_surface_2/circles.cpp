//! \file examples/Arrangement_on_surface_2/circles.cpp
// Constructing an arrangement of circles using the circle-segment traits.

#include <CGAL/draw_arrangement_2.h>

#include "arr_circular.h"

int main() {
  Arrangement arr;

  for(double x = 0; x < 4; x++) {
    for(double y = 0; y < 4; y++) {
      // Insert a circle with center (x, y) and radius 1.
      Circle c({x * 2, y * 2}, 6);
      CGAL::insert(arr, c);
    }
  }

  CGAL::draw(arr);
  return 0;
}
