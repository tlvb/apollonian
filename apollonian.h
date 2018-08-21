#include <iostream>
#include <complex>

typedef long double R;
typedef std::complex<R> C;

inline R fourth_curvature_p(R k1, R k2, R k3)
{
  return k1 + k2 + k3 + static_cast<R>(2)*sqrt(k1*k2 + k2*k3 + k3*k1);
}
inline R fourth_curvature_n(R k1, R k2, R k3)
{
  return k1 + k2 + k3 - static_cast<R>(2)*sqrt(k1*k2 + k2*k3 + k3*k1);
}
inline C fourth_center_p(R k1, const C& z1, R k2, const C& z2, R k3, const C& z3, R k4)
{
  return ( z1*k1 + z2*k2 + z3*k3 + static_cast<R>(2)*sqrt(k1*k2*z1*z2 + k2*k3*z2*z3 + k3*k1*z3*z1 ) ) / k4;
}
inline C fourth_center_n(R k1, const C& z1, R k2, const C& z2, R k3, const C& z3, R k4)
{
  return ( z1*k1 + z2*k2 + z3*k3 - static_cast<R>(2)*sqrt(k1*k2*z1*z2 + k2*k3*z2*z3 + k3*k1*z3*z1 ) ) / k4;
}
struct apocirc
{
  R k; // curvature
  C z; // center
  R r2; // square radius = 1/square curvature
  apocirc(void);
  apocirc(const C& pos, const R curvature);
  apocirc(const apocirc& c1, const apocirc& c2, const apocirc& c3);
  /*
  apocirc(const apocirc& c1, const apocirc& c2, const apocirc& c3, R rot)
  {
    const R k1 = c1.k * 0.8;
    const R k2 = c2.k * 0.8;
    const R k3 = c3.k * 0.8;
    const C z1 = c1.z*exp(C(0, rot))+C(10,10);
    const C z2 = c2.z*exp(C(0, rot))+C(10,10);
    const C z3 = c3.z*exp(C(0, rot))+C(10,10);
    k = fourth_curvature_p(k1, k2, k3);
    z = fourth_center_p(k1, z1, k2, z2, k3, z3, k)-C(10,10);
    r2 = static_cast<R>(1)/(k*k);
  }
  */
  R square_distance(const C& point) const;
  bool contains(const C& point) const;
  void mapcolor(const C& point, uint8_t& red, uint8_t& green, uint8_t& blue);
};

std::ostream& operator<<(std::ostream& os, const apocirc& a);

bool point_leftof_line(const C& point, const C& z1, const C& z2);
bool point_inside_triangle(const C& point, const C& z1, const C& z2, const C& z3);
class aponode_i;
// peripheral aponode, the apocirc has contact with the bounding apocirc
class aponode_p
{
  apocirc c;
  const apocirc& _bpc;  // bounding parent apocirc
  const apocirc& _ipc1; // interior parent apocirc 1
  const apocirc& _ipc2; //                         2
  aponode_p* _pcn1;     // peripheral child node 1
  aponode_p* _pcn2;     //                       2
  aponode_i* _icn;      // interior child node
 public:
  aponode_p(const apocirc& bpc, const apocirc& ipc1, const apocirc& ipc2);
  ~aponode_p(void);
  void create_children(int levels);
  void create_children_by_max_curvature(R max_curvature);
  R square_distance(const C& point);
  void mapcolor(const C& point, uint8_t& red, uint8_t& green, uint8_t& blue);
  friend std::ostream& operator<<(std::ostream& os, const aponode_p& n);
};
//std::ostream& operator<<(std::ostream& os, const aponode_p& n);
// interior aponode, the apocirc has no contact with the bounding apocirc
class aponode_i
{
  apocirc c;
  const apocirc& _ipc1; // interior parent apocirc 1
  const apocirc& _ipc2; //                         2
  const apocirc& _ipc3; //                         3
  aponode_i* _icn1;  // interior child node 1
  aponode_i* _icn2;  //                     2
  aponode_i* _icn3;  //                     3
 public:
  aponode_i(const apocirc& ipc1, const apocirc& ipc2, const apocirc& ipc3);
  ~aponode_i(void);
  void create_children(int levels);
  void create_children_by_max_curvature(R max_curvature);
  R square_distance(const C& point);
  void mapcolor(const C& point, uint8_t& red, uint8_t& green, uint8_t& blue);
  friend std::ostream& operator<<(std::ostream& os, const aponode_i& n);
};
std::ostream& operator<<(std::ostream& os, const aponode_i& n);
class apollonian_tree
{
  apocirc a;
  apocirc b;
  apocirc c;
  apocirc d;
  apocirc e;

  aponode_p n1;
  aponode_p n2;
  aponode_p n3;
  aponode_p n4;
  aponode_i n5;
  aponode_i n6;

 public:
  apollonian_tree(void);
  apollonian_tree(R k, const C& z);
  void create_children(int levels);
  void create_children_by_max_curvature(R max_curvature);
  bool contains(const C& point);
  void mapcolor(const C& point, uint8_t& r, uint8_t& g, uint8_t& b);
  
  friend std::ostream& operator<<(std::ostream& os, const apollonian_tree& t);

 private:
  apollonian_tree(R ka, const C& za, R kb, const C& zb, R kc, const C& zc);
  apollonian_tree(R ka, const C& za, R kb, const C& zb, R kc, const C& zc, R kp, R kn);
  apollonian_tree(R ka, const C& za, R kb, const C& zb, R kc, const C& zc, R kp, const C& zp, R kn, const C& zn);
};
