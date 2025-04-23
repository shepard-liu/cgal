
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/draw_arrangement_2.h>
#include <CGAL/draw_polygon_set_2.h>
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;

int main() {
  CGAL::Polygon_set_2<Kernel> ps;

  CGAL::Polygon_2<Kernel> p;
  p.push_back(Kernel::Point_2(0, 0));
  p.push_back(Kernel::Point_2(2, 0));
  p.push_back(Kernel::Point_2(2, 2));
  p.push_back(Kernel::Point_2(0, 2));

  CGAL::Polygon_2<Kernel> hole;
  hole.push_back(Kernel::Point_2(0.5, 0.5));
  hole.push_back(Kernel::Point_2(1.5, 0.5));
  hole.push_back(Kernel::Point_2(1.5, 1.5));
  hole.push_back(Kernel::Point_2(0.5, 1.5));
  hole.reverse_orientation();

  CGAL::Polygon_with_holes_2<Kernel> pwh(p);
  pwh.add_hole(hole);

  ps.insert(p);

  CGAL::draw(ps);

  return 0;
}
