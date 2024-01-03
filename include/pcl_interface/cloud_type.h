#ifndef __MY_CLOUD_TYPE_H__
#define __MY_CLOUD_TYPE_H__

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_macros.h>
#include <pcl/kdtree/kdtree_flann.h>

#define PCL_ADD_UNION_A1 \
  union EIGEN_ALIGN16 { \
    float angle[1]; \
    struct { \
      float a0; \
    }; \
  };

#define PCL_ADD_A1 \
  PCL_ADD_UNION_A1

struct EIGEN_ALIGN16 PN2V1A
{
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  float curvature;
  float xx;
  float xy;
  float xz;
  float zx;
  float zy;
  float zz;
  PCL_ADD_A1;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PN2V1A,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, curvature, curvature)
                                   (float, xx, xx)
                                   (float, xy, xy)
                                   (float, xz, xz)
                                   (float, zx, zx)
                                   (float, zy, zy)
                                   (float, zz, zz)
                                   (float, a0, a0)
)

#define PCL_ADD_UNION_A4 \
  union EIGEN_ALIGN16 { \
    float angle[4]; \
    struct { \
      float a0; \
      float a1; \
      float a2; \
      float a3; \
    }; \
  };

#define PCL_ADD_A4 \
  PCL_ADD_UNION_A4

struct EIGEN_ALIGN16 PN2V4A
{
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  float curvature;
  float xx;
  float xy;
  float xz;
  float zx;
  float zy;
  float zz;
  PCL_ADD_A4;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PN2V4A,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, curvature, curvature)
                                   (float, xx, xx)
                                   (float, xy, xy)
                                   (float, xz, xz)
                                   (float, zx, zx)
                                   (float, zy, zy)
                                   (float, zz, zz)
                                   (float, a0, a0)
                                   (float, a1, a1)
                                   (float, a2, a2)
                                   (float, a3, a3)
)

#define PCL_ADD_UNION_A5 \
  union EIGEN_ALIGN16 { \
    float angle[5]; \
    struct { \
      float a0; \
      float a1; \
      float a2; \
      float a3; \
      float a4; \
    }; \
  };

#define PCL_ADD_A5 \
  PCL_ADD_UNION_A5

struct EIGEN_ALIGN16 PN2V5A
{
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  float curvature;
  float xx;
  float xy;
  float xz;
  float zx;
  float zy;
  float zz;
  PCL_ADD_A5;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PN2V5A,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, curvature, curvature)
                                   (float, xx, xx)
                                   (float, xy, xy)
                                   (float, xz, xz)
                                   (float, zx, zx)
                                   (float, zy, zy)
                                   (float, zz, zz)
                                   (float, a0, a0)
                                   (float, a1, a1)
                                   (float, a2, a2)
                                   (float, a3, a3)
                                   (float, a4, a4)
)

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr  xyzPtr;
typedef pcl::PointCloud<pcl::PointXYZ>       xyz;
typedef pcl::KdTreeFLANN<pcl::PointXYZ>      treeXYZ;
typedef pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr treeXYZPtr;

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB>      rgb;
typedef pcl::KdTreeFLANN<pcl::PointXYZRGB>     treeRGB;

typedef pcl::PointCloud<pcl::PointNormal>::Ptr  pnPtr;
typedef pcl::PointCloud<pcl::PointNormal>       pn;
typedef pcl::KdTreeFLANN<pcl::PointNormal>      treePN;
typedef pcl::KdTreeFLANN<pcl::PointNormal>::Ptr treePNPtr;

typedef pcl::PointCloud<PN2V1A>::Ptr  pn2v1aPtr;
typedef pcl::PointCloud<PN2V1A>       pn2v1a;
typedef pcl::KdTreeFLANN<PN2V1A>      treePn2v1a;
typedef pcl::KdTreeFLANN<PN2V1A>::Ptr treePn2v1aPtr;

typedef pcl::PointCloud<PN2V4A>::Ptr  pn2v4aPtr;
typedef pcl::PointCloud<PN2V4A>       pn2v4a;
typedef pcl::KdTreeFLANN<PN2V4A>      treePn2v4a;
typedef pcl::KdTreeFLANN<PN2V4A>::Ptr treePn2v4aPtr;

typedef pcl::PointCloud<PN2V5A>::Ptr  pn2v5aPtr;
typedef pcl::PointCloud<PN2V5A>       pn2v5a;
typedef pcl::KdTreeFLANN<PN2V5A>      treePn2v5a;
typedef pcl::KdTreeFLANN<PN2V5A>::Ptr treePn2v5aPtr;

#endif
