//提供伪代码

#ifndef _PCL_CLOUD_STITCH_
#define _PCL_CLOUD_STITCH_

//A雷达
struct PointXYZA{
  PCL_ADD_POINT4D   
  float intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
} EIGEN_ALIGN16;                   

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZA,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
)

//B雷达
struct PointXYZB{
  PCL_ADD_POINT4D 
  float intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
} EIGEN_ALIGN16;                   

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZB,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
)

//点云总结构结构
struct PointXYZSTITCH {
  PCL_ADD_POINT4D   
  float intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
} EIGEN_ALIGN16;                   

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZSTITCH,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
)


void PclPointStitchA(pcl::PointCloud<PointXYZSTITCH>::Ptr& stitch,const pcl::PointCloud<PointXYZA>::Ptr& pcl_a,int &index);
void PclPointStitchB(pcl::PointCloud<PointXYZSTITCH>::Ptr& stitch,const pcl::PointCloud<PointXYZB>::Ptr& pcl_b,int &index);

#endif