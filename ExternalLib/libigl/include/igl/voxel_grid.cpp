// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2016 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "voxel_grid.h"
#include "grid.h"

template <
  typename Scalar,
  typename DerivedGV,
  typename Derivedside>
IGL_INLINE void igl::voxel_grid(
  const Eigen::AlignedBox<Scalar,3> & box,
  const int in_s,
  const int pad_count,
  Eigen::PlainObjectBase<DerivedGV> & GV,
  Eigen::PlainObjectBase<Derivedside> & side)
{
  using namespace Eigen;
  using namespace std;
  typename DerivedGV::Index si = -1;
  side.resize(1, 3);
  box.diagonal().maxCoeff(&si);
  //DerivedGV::Index si = 0;
  //assert(si>=0);
  const Scalar s_len = box.diagonal()(si);
  assert(in_s>(pad_count*2+1) && "s should be > 2*pad_count+1");
  const Scalar s = in_s - 2*pad_count;
  side(si) = s;
  for(int i = 0;i<3;i++)
  {
    if(i!=si)
    {
      side(i) = std::ceil(s * (box.max()(i)-box.min()(i))/s_len);
    }
  }
  side.array() += 2*pad_count;
  grid(side,GV);
  // A *    p/s  + B = min
  // A * (1-p/s) + B = max
  // B = min - A * p/s
  // A * (1-p/s) + min - A * p/s = max
  // A * (1-p/s) - A * p/s = max-min
  // A * (1-2p/s) = max-min
  // A  = (max-min)/(1-2p/s)
  const Array<Scalar,3,1> ps=
    (Scalar)(pad_count)/(side.transpose().template cast<Scalar>().array()-1.);
  const Array<Scalar,3,1> A = box.diagonal().array()/(1.0-2.*ps);
  //// This would result in an "anamorphic", but perfectly fit grid:
  //const Array<Scalar,3,1> B = box.min().array() - A.array()*ps;
  //GV.array().rowwise() *= A.transpose();
  //GV.array().rowwise() += B.transpose();
  // Instead scale by largest factor and move to match center
  typename Array<Scalar,3,1>::Index ai = -1;
  Scalar a = A.maxCoeff(&ai);
  const Array<Scalar,1,3> ratio =
    a*(side.template cast<Scalar>().array()-1.0)/(Scalar)(side(ai)-1.0);
  GV.array().rowwise() *= ratio;
  const Eigen::Matrix<Scalar,1,3> offset = (box.center().transpose()-GV.colwise().mean()).eval();
  GV.rowwise() += offset;
}

template <
  typename DerivedV,
  typename DerivedGV,
  typename Derivedside>
IGL_INLINE void igl::voxel_grid(
  const Eigen::MatrixBase<DerivedV> & V, 
  const typename DerivedV::Scalar offset,
  const int s,
  const int pad_count,
  Eigen::PlainObjectBase<DerivedGV> & GV,
  Eigen::PlainObjectBase<Derivedside> & side)
{
  typedef typename DerivedV::Scalar Scalar;
  Eigen::AlignedBox<Scalar,3> box;
  typedef Eigen::Matrix<Scalar,1,3> RowVector3S;
  assert(V.cols() == 3 && "V must contain positions in 3D");
  RowVector3S min_ext = V.colwise().minCoeff().array() - offset;
  RowVector3S max_ext = V.colwise().maxCoeff().array() + offset;
  box.extend(min_ext.transpose());
  box.extend(max_ext.transpose());
  return igl::voxel_grid(box,s,1,GV,side);
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template instantiation
// generated by autoexplicit.sh
template void igl::voxel_grid<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, 3, 1, -1, 3>, Eigen::Matrix<int, 1, 3, 1, 1, 3> >(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::Matrix<double, -1, -1, 0, -1, -1>::Scalar, int, int, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, 1, 3, 1, 1, 3> >&);
// generated by autoexplicit.sh
template void igl::voxel_grid<Eigen::Matrix<float, -1, 3, 1, -1, 3>, Eigen::Matrix<float, -1, -1, 0, -1, -1>, Eigen::Matrix<int, 1, 3, 1, 1, 3> >(Eigen::MatrixBase<Eigen::Matrix<float, -1, 3, 1, -1, 3> > const&, Eigen::Matrix<float, -1, 3, 1, -1, 3>::Scalar, int, int, Eigen::PlainObjectBase<Eigen::Matrix<float, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, 1, 3, 1, 1, 3> >&);
// generated by autoexplicit.sh
template void igl::voxel_grid<Eigen::Matrix<double, -1, 3, 1, -1, 3>, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, 1, 3, 1, 1, 3> >(Eigen::MatrixBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> > const&, Eigen::Matrix<double, -1, 3, 1, -1, 3>::Scalar, int, int, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, 1, 3, 1, 1, 3> >&);
// generated by autoexplicit.sh
template void igl::voxel_grid<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, 1, 3, 1, 1, 3> >(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::Matrix<double, -1, -1, 0, -1, -1>::Scalar, int, int, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, 1, 3, 1, 1, 3> >&);
// generated by autoexplicit.sh
template void igl::voxel_grid<float, Eigen::Matrix<float, -1, -1, 0, -1, -1>, Eigen::Matrix<int, 3, 1, 0, 3, 1> >(Eigen::AlignedBox<float, 3> const&, int, int, Eigen::PlainObjectBase<Eigen::Matrix<float, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, 3, 1, 0, 3, 1> >&);
// generated by autoexplicit.sh
template void igl::voxel_grid<float, Eigen::Matrix<float, -1, -1, 0, -1, -1>, Eigen::Matrix<int, 1, 3, 1, 1, 3> >(Eigen::AlignedBox<float, 3> const&, int, int, Eigen::PlainObjectBase<Eigen::Matrix<float, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, 1, 3, 1, 1, 3> >&);
template void igl::voxel_grid<float, Eigen::Matrix<float, -1, 3, 0, -1, 3>, Eigen::Matrix<int, 3, 1, 0, 3, 1> >(Eigen::AlignedBox<float, 3> const&, int, int, Eigen::PlainObjectBase<Eigen::Matrix<float, -1, 3, 0, -1, 3> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, 3, 1, 0, 3, 1> >&);
template void igl::voxel_grid<float, Eigen::Matrix<float, -1, 3, 1, -1, 3>, Eigen::Matrix<int, 3, 1, 0, 3, 1> >(Eigen::AlignedBox<float, 3> const&, int, int, Eigen::PlainObjectBase<Eigen::Matrix<float, -1, 3, 1, -1, 3> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, 3, 1, 0, 3, 1> >&);
template void igl::voxel_grid<double, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, 3, 1, 0, 3, 1> >(Eigen::AlignedBox<double, 3> const&, int, int, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, 3, 1, 0, 3, 1> >&);
template void igl::voxel_grid<double, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, 1, 3, 1, 1, 3> >(Eigen::AlignedBox<double, 3> const&, int, int, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, 1, 3, 1, 1, 3> >&);
#endif
