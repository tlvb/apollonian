#include <iostream>
#include <complex>
#include "apollonian.h"

const C calc_offset(10,10);
apocirc::apocirc(void) : k(-1), z(0,0), r2(1) {}
apocirc::apocirc(const C& pos, const R curvature) : k(curvature), z(pos), r2(static_cast<R>(1)/(k*k)) {}
apocirc::apocirc(const apocirc& c1, const apocirc& c2, const apocirc& c3)
{
  const R k1 = c1.k;
  const R k2 = c2.k;
  const R k3 = c3.k;
  const C z1 = c1.z+calc_offset;
  const C z2 = c2.z+calc_offset;
  const C z3 = c3.z+calc_offset;
  k = fourth_curvature_p(k1, k2, k3);
  z = fourth_center_p(k1, z1, k2, z2, k3, z3, k)-calc_offset;
  r2 = static_cast<R>(1)/(k*k);
}
R apocirc::square_distance(const C& point) const
{
  return norm(z-point) - r2;
}
bool apocirc::contains(const C& point) const
{
  return square_distance(point) <= 0;
}
void apocirc::mapcolor(const C& point, uint8_t& red, uint8_t& green, uint8_t& blue)
{
  R d2 = square_distance(point);
  if (d2 < 0) {
    red = green = blue = static_cast<uint8_t>(-255*d2/r2);
  }
}

std::ostream& operator<<(std::ostream& os, const apocirc& a)
{
  os << "circle(" << a.z.real() << ", " << a.z.imag() << ", " << (1/a.k) << ");" << std::endl;
  return os;
}

bool point_leftof_line(const C& point, const C& z1, const C& z2)
{
  return (point.real()-z1.real())*(z2.imag()-z1.imag()) < (point.imag()-z1.imag())*(z2.real()-z1.real());
}
bool point_inside_triangle(const C& point, const C& z1, const C& z2, const C& z3)
{
  const bool b1 = point_leftof_line(point, z1, z2);
  const bool b2 = point_leftof_line(point, z2, z3);
  const bool b3 = point_leftof_line(point, z3, z1);
  return (b1 == b2) && (b2 == b3);
}
aponode_p::aponode_p(const apocirc& bpc, const apocirc& ipc1, const apocirc& ipc2) :
  c(bpc, ipc1, ipc2),
  _bpc(bpc),
  _ipc1(ipc1),
  _ipc2(ipc2),
  _pcn1(nullptr),
  _pcn2(nullptr),
  _icn(nullptr)
{}
aponode_p::~aponode_p(void)
{
  if (_pcn1 != nullptr) { delete _pcn1; }
  if (_pcn2 != nullptr) { delete _pcn2; }
  if (_icn  != nullptr) { delete _icn;  }
}
void aponode_p::create_children(int levels)
{
  if (_pcn1 == nullptr) { _pcn1 = new aponode_p(_bpc, _ipc1, c);  }
  if (_pcn2 == nullptr) { _pcn2 = new aponode_p(_bpc, c, _ipc2);  }
  if (_icn == nullptr ) { _icn  = new aponode_i(_ipc1, _ipc2, c); }
  if (levels > 1) {
    _pcn1->create_children(levels-1);
    _pcn2->create_children(levels-1);
    _icn->create_children(levels-1);
  }
}
void aponode_p::create_children_by_max_curvature(R max_curvature)
{
  if (c.k < max_curvature) {
    if (_pcn1 == nullptr) { _pcn1 = new aponode_p(_bpc, _ipc1, c);  }
    if (_pcn2 == nullptr) { _pcn2 = new aponode_p(_bpc, c, _ipc2);  }
    if (_icn == nullptr ) { _icn  = new aponode_i(_ipc1, _ipc2, c); }
    _pcn1->create_children_by_max_curvature(max_curvature);
    _pcn2->create_children_by_max_curvature(max_curvature);
    _icn->create_children_by_max_curvature(max_curvature);
  }
}
R aponode_p::square_distance(const C& point) {
  R d2_c    = c.square_distance(point);

  if (d2_c < 0) {
    return d2_c;
  }
  else if (point_inside_triangle(point, _ipc1.z, _ipc2.z, c.z)) {
    if (_icn != nullptr) {
      R d2_icn = _icn->square_distance(point);
      return d2_icn < d2_c ? d2_icn : d2_c;
    }
    else {
      return d2_c;
    }
  }
  else if (point_leftof_line(point, _bpc.z, c.z)) {
    if (_pcn1 != nullptr) {
      R d2_pcn1 = _pcn1->square_distance(point);
      return d2_pcn1 < d2_c ? d2_pcn1 : d2_c;
    }
    else {
      return d2_c;
    }
  }
  else {
    if (_pcn2 != nullptr) {
      R d2_pcn2 = _pcn2->square_distance(point);
      return d2_pcn2 < d2_c ? d2_pcn2 : d2_c;
    }
    else {
      return d2_c;
    }
  }
}
void aponode_p::mapcolor(const C& point, uint8_t& red, uint8_t& green, uint8_t& blue)
{
  if (c.contains(point)) {
    c.mapcolor(point, red, green, blue);
  }
  else if (point_inside_triangle(point, _ipc1.z, c.z, _ipc2.z)) {
    if (_icn != nullptr) {
      _icn->mapcolor(point, red, green, blue);
    }
  }
  else {
    if (point_leftof_line(point, _bpc.z, c.z)) {
      if (_pcn1 != nullptr) {
        _pcn1->mapcolor(point, red, green, blue);
      }
    }
    else {
      if (_pcn2 != nullptr) {
        _pcn2->mapcolor(point, red, green, blue);
      }
    }
  }
}
std::ostream& operator<<(std::ostream& os, const aponode_p& n)
{
  os << n.c;
  if (n._pcn1 != nullptr) { os << *n._pcn1; }
  if (n._pcn2 != nullptr) { os << *n._pcn2; }
  if (n._icn  != nullptr) { os << *n._icn;  }
  return os;
}
aponode_i::aponode_i(const apocirc& ipc1, const apocirc& ipc2, const apocirc& ipc3) :
  c(ipc1, ipc2, ipc3),
  _ipc1(ipc1),
  _ipc2(ipc2),
  _ipc3(ipc3),
  _icn1(nullptr),
  _icn2(nullptr),
  _icn3(nullptr)
{}
aponode_i::~aponode_i(void)
{
  if (_icn1 != nullptr) { delete _icn1; }
  if (_icn2 != nullptr) { delete _icn2; }
  if (_icn3 != nullptr) { delete _icn3; }
}
void aponode_i::create_children(int levels)
{
  if (_icn1 == nullptr) { _icn1 = new aponode_i(_ipc1, _ipc2, c); }
  if (_icn2 == nullptr) { _icn2 = new aponode_i(_ipc1, _ipc3, c); }
  if (_icn3 == nullptr) { _icn3 = new aponode_i(_ipc2, _ipc3, c); }
  if (levels > 1) {
    _icn1->create_children(levels-1);
    _icn2->create_children(levels-1);
    _icn3->create_children(levels-1);
  }
}
void aponode_i::create_children_by_max_curvature(R max_curvature)
{
  if (c.k < max_curvature) {
    if (_icn1 == nullptr) { _icn1 = new aponode_i(_ipc1, _ipc2, c); }
    if (_icn2 == nullptr) { _icn2 = new aponode_i(_ipc1, _ipc3, c); }
    if (_icn3 == nullptr) { _icn3 = new aponode_i(_ipc2, _ipc3, c); }
    _icn1->create_children_by_max_curvature(max_curvature);
    _icn2->create_children_by_max_curvature(max_curvature);
    _icn3->create_children_by_max_curvature(max_curvature);
  }
}
R aponode_i::square_distance(const C& point) {
  return 0;
}
void aponode_i::mapcolor(const C& point, uint8_t& red, uint8_t& green, uint8_t& blue)
{
  if (c.contains(point)) {
    c.mapcolor(point, red, green, blue);
  }
  else if (point_inside_triangle(point, _ipc1.z, _ipc2.z, c.z)) {
    // _icn1
    if (_icn1 != nullptr) {
      _icn1->mapcolor(point, red, green, blue);
    }
  }
  else if (point_inside_triangle(point, _ipc1.z, _ipc3.z, c.z)) {
    // _icn2
    if (_icn2 != nullptr) {
      _icn2->mapcolor(point, red, green, blue);
    }
  }
  else {
    // _icn3
    if (_icn3 != nullptr) {
      _icn3->mapcolor(point, red, green, blue);
    }
  }
}
std::ostream& operator<<(std::ostream& os, const aponode_i& n)
{
  os << n.c;
  if (n._icn1 != nullptr) { os << *n._icn1; }
  if (n._icn2 != nullptr) { os << *n._icn2; }
  if (n._icn3 != nullptr) { os << *n._icn3; }
  return os;
}



/*
  appolonian:
  bounding circle: negative curvature
  incircle a
  incircle b
  makes up S

  for all subsets s of S of size 3 make one new circle tangent to all three parents s
  two circles will be possible:
  if only three circles exist so far:
  generate both possible solutions, one on ach side of the
  degenerate triangle (line) formed by the parents' centers
  otherwise:
  select the solution that has the largest positive curvature




  curvature:
  k4 = k1 + k2 + k3 +- sqrt( k1k2 + k2k3 + k3k1)

  center (complex)

  z4 = ( z1k1 +z2k2 + z3k3 +- sqrt( k1k2z1z2 + k2k3z2z3 + k3k1z3z1 ) ) / k4
*/


apollonian_tree::apollonian_tree(void) :
  apollonian_tree(-1.0, C(0.0, 0.0))
{}
apollonian_tree::apollonian_tree(R k, const C& z) :
  apollonian_tree(
                  k, z,
                  k*-2, z+0.5/k,
                  k*-2, z-0.5/k)
{}
apollonian_tree::apollonian_tree(R ka, const C& za, R kb, const C& zb, R kc, const C& zc) :
  apollonian_tree(
                  ka, za,
                  kb, zb,
                  kc, zc,
                  fourth_curvature_p(ka, kb, kc),
                  fourth_curvature_n(ka, kb, kc))
{}
apollonian_tree::apollonian_tree(R ka, const C& za, R kb, const C& zb, R kc, const C& zc, R kp, R kn) :
  a(za, ka),
  b(zb, kb),
  c(zc, kc),
  d(fourth_center_p(ka, za, kb, zb, kc, zc, kp), kp),
  e(fourth_center_n(ka, za, kb, zb, kc, zc, kn), kn),
  n1(a, b, d),
  n2(a, d, c),
  n3(a, c, e),
  n4(a, e, b),
  n5(b, d, c),
  n6(c, e, b)
{}
void apollonian_tree::create_children(int levels)
{
  if (levels > 3) {
    n1.create_children(levels-3);
    n2.create_children(levels-3);
    n3.create_children(levels-3);
    n4.create_children(levels-3);
    n5.create_children(levels-3);
    n6.create_children(levels-3);
  }
}
void apollonian_tree::create_children_by_max_curvature(R max_curvature)
{
  n1.create_children_by_max_curvature(max_curvature);
  n2.create_children_by_max_curvature(max_curvature);
  n3.create_children_by_max_curvature(max_curvature);
  n4.create_children_by_max_curvature(max_curvature);
  n5.create_children_by_max_curvature(max_curvature);
  n6.create_children_by_max_curvature(max_curvature);
}
bool apollonian_tree::contains(const C& point)
{
  if (a.contains(point)) {
    if (b.contains(point)) {
      return false;
    }
    else if (c.contains(point)) {
      return false;
    }
    else {
      if (point_leftof_line(point, b.z, c.z)) {
        if (d.contains(point)) {
          return false;
        }
        else {
          if (point_inside_triangle(point, b.z, d.z, c.z)) {
            return false;
          }
          else if (point_leftof_line(point, a.z, d.z)) {
            return false;
          }
          else {
            return true;
          }
        }
      }
      else {
        if (e.contains(point)) {
          return false;
        }
        else {
          if (point_inside_triangle(point, c.z, e.z, b.z)) {
            return false;
          }
          else if (point_leftof_line(point, a.z, e.z)) {
            return false;
          }
          else {
            return true;
          }
        }
      }
    }
  }
  else {
    return false;
  }
}
void apollonian_tree::mapcolor(const C& point, uint8_t& red, uint8_t& green, uint8_t& blue)
{
  if (a.contains(point)) {
    red = green = blue = 0;
    if (b.contains(point)) {
      b.mapcolor(point, red, green, blue);
    }
    else if (c.contains(point)) {
      c.mapcolor(point, red, green, blue);
    }
    else {
      if (point_leftof_line(point, b.z, c.z)) {
        if (d.contains(point)) {
          d.mapcolor(point, red, green, blue);
        }
        else {
          if (point_inside_triangle(point, b.z, d.z, c.z)) {
            n5.mapcolor(point, red, green, blue);
          }
          else if (point_leftof_line(point, a.z, d.z)) {
            n1.mapcolor(point, red, green, blue);
          }
          else {
            n2.mapcolor(point, red, green, blue);
          }
        }
      }
      else {
        if (e.contains(point)) {
          e.mapcolor(point, red, green, blue);
        }
        else {
          if (point_inside_triangle(point, c.z, e.z, b.z)) {
            n6.mapcolor(point, red, green, blue);
          }
          else if (point_leftof_line(point, a.z, e.z)) {
            n3.mapcolor(point, red, green, blue);
          }
          else {
            n4.mapcolor(point, red, green, blue);
          }
        }
      }
    }
  }
  else {
    red = green = blue = 0;//rand();
  }
}
std::ostream& operator<<(std::ostream& os, const apollonian_tree& t)
{
  os << t.a;
  os << t.b;
  os << t.c;
  os << t.d;
  os << t.e;
  os << t.n1 << t.n2 << t.n3 << t.n4 << t.n5 << t.n6;
  return os;
}

int main(void) {

  apollonian_tree t;
  std::cerr << "% generating..." << std::endl;
  t.create_children(15);
  t.create_children_by_max_curvature(1000000);

  /*
    std::cout << "close all" << std::endl;
    std::cout << t;
    std::cout << "axis square" << std::endl;
    std::cout << "axis equal" << std::endl;
  */

  std::cerr << "% writing..." << std::endl;

  // C file io is superior
  FILE *f = fopen("apollonian.ppm", "w");
  // FILE *f = stdout;
  size_t w = 16384;
  size_t h = 16384;
  R maxdim = static_cast<R>(std::max(w, h));
  fprintf(f, "P6\n%zu %zu\n255\n", w, h);
  for (size_t y=0; y<h; ++y) {
    if ((y & 0x3ff) == 0) {
      std::cerr << "% row " << y << std::endl;
    }
    size_t yy = h-y;
    for (size_t x=0; x<w; ++x) {
      uint8_t red, green, blue;
      red = green = blue = 0;
      t.mapcolor((static_cast<R>(2.1)*C(x, yy)/maxdim - C(1.05,1.05)), red, green, blue);
      fprintf(f, "%c%c%c", red, green, blue);
    }
  }
  fclose(f);
  std::cerr << "% writing done" << std::endl;
  //std::cerr << t;
}
