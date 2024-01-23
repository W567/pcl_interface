#ifndef __MY_CLOUD_TYPE_H__
#define __MY_CLOUD_TYPE_H__

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_macros.h>
#include <pcl/kdtree/kdtree_flann.h>


#define COMMON_SLOT \
  PCL_ADD_POINT4D; \
  PCL_ADD_NORMAL4D; \
  float curvature; \
  float xx; \
  float xy; \
  float xz; \
  float zx; \
  float zy; \
  float zz;


#define COMMON_DATA \
  (float, x, x) \
  (float, y, y) \
  (float, z, z) \
  (float, normal_x, normal_x) \
  (float, normal_y, normal_y) \
  (float, normal_z, normal_z) \
  (float, curvature, curvature) \
  (float, xx, xx) \
  (float, xy, xy) \
  (float, xz, xz) \
  (float, zx, zx) \
  (float, zy, zy) \
  (float, zz, zz)


#define PCL_ADD_A1 \
  union EIGEN_ALIGN16 { \
    float angle[1]; \
    struct { \
      float a0; \
    }; \
  };


#define PCL_ADD_A2 \
  union EIGEN_ALIGN16 { \
    float angle[2]; \
    struct { \
      float a0; \
      float a1; \
    }; \
  };


#define PCL_ADD_A3 \
  union EIGEN_ALIGN16 { \
    float angle[3]; \
    struct { \
      float a0; \
      float a1; \
      float a2; \
    }; \
  };


#define PCL_ADD_A4 \
  union EIGEN_ALIGN16 { \
    float angle[4]; \
    struct { \
      float a0; \
      float a1; \
      float a2; \
      float a3; \
    }; \
  };


#define PCL_ADD_A5 \
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


#define CLOUD_STRUCT(NAME, ADD) \
  struct EIGEN_ALIGN16 NAME \
  { \
    COMMON_SLOT \
    ADD \
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW \
  };


CLOUD_STRUCT (PN2V1A, PCL_ADD_A1)
POINT_CLOUD_REGISTER_POINT_STRUCT (PN2V1A,
                                   COMMON_DATA
                                   (float, a0, a0)
                                  )


CLOUD_STRUCT (PN2V2A, PCL_ADD_A2)
POINT_CLOUD_REGISTER_POINT_STRUCT (PN2V2A,
                                   COMMON_DATA
                                   (float, a0, a0)
                                   (float, a1, a1)
                                  )


CLOUD_STRUCT (PN2V3A, PCL_ADD_A3)
POINT_CLOUD_REGISTER_POINT_STRUCT (PN2V3A,
                                   COMMON_DATA
                                   (float, a0, a0)
                                   (float, a1, a1)
                                   (float, a2, a2)
                                  )


CLOUD_STRUCT (PN2V4A, PCL_ADD_A4)
POINT_CLOUD_REGISTER_POINT_STRUCT (PN2V4A,
                                   COMMON_DATA
                                   (float, a0, a0)
                                   (float, a1, a1)
                                   (float, a2, a2)
                                   (float, a3, a3)
                                  )


CLOUD_STRUCT (PN2V5A, PCL_ADD_A5)
POINT_CLOUD_REGISTER_POINT_STRUCT (PN2V5A,
                                   COMMON_DATA
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

typedef pcl::PointCloud<PN2V2A>::Ptr  pn2v2aPtr;
typedef pcl::PointCloud<PN2V2A>       pn2v2a;
typedef pcl::KdTreeFLANN<PN2V2A>      treePn2v2a;
typedef pcl::KdTreeFLANN<PN2V2A>::Ptr treePn2v2aPtr;

typedef pcl::PointCloud<PN2V3A>::Ptr  pn2v3aPtr;
typedef pcl::PointCloud<PN2V3A>       pn2v3a;
typedef pcl::KdTreeFLANN<PN2V3A>      treePn2v3a;
typedef pcl::KdTreeFLANN<PN2V3A>::Ptr treePn2v3aPtr;

typedef pcl::PointCloud<PN2V4A>::Ptr  pn2v4aPtr;
typedef pcl::PointCloud<PN2V4A>       pn2v4a;
typedef pcl::KdTreeFLANN<PN2V4A>      treePn2v4a;
typedef pcl::KdTreeFLANN<PN2V4A>::Ptr treePn2v4aPtr;

typedef pcl::PointCloud<PN2V5A>::Ptr  pn2v5aPtr;
typedef pcl::PointCloud<PN2V5A>       pn2v5a;
typedef pcl::KdTreeFLANN<PN2V5A>      treePn2v5a;
typedef pcl::KdTreeFLANN<PN2V5A>::Ptr treePn2v5aPtr;

#endif
