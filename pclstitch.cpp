#include "pclstitch.h"

//算出总点数，分配总空间数
int main() {
    int total_points = 10000;
    pcl::PointCloud<PointXYZSTITCH>::Ptr stitch(new pcl::PointCloud<PointXYZSTITCH>);
    stitch->points.resize(total_points); //
    stitch->width = total_points;
    stitch->height = 1;
    int index = 0

    pcl::PointCloud<PointXYZA>::Ptr lidar_a_source(new pcl::PointCloud<PointXYZA>);
    pcl::PointCloud<PointXYZA>::Ptr lidar_a(new pcl::PointCloud<PointXYZA>);
    //矩阵转换:lidar_a_trans a相对某个坐标系的转换矩阵
    pcl::transformPointCloud(*lidar_a_source, *lidar_a, Eigen::Matrix4d lidar_a_trans);
    PclPointStitchA(stitch,lidar_a,index);

    pcl::PointCloud<PointXYZB>::Ptr lidar_b_source(new pcl::PointCloud<PointXYZB>);
    pcl::PointCloud<PointXYZB>::Ptr lidar_b(new pcl::PointCloud<PointXYZB>);
    //矩阵转换:lidar_b_trans b相对某个坐标系的转换矩阵
    pcl::transformPointCloud(*lidar_b_source, *lidar_b, Eigen::Matrix4d lidar_b_trans);
    PclPointStitchB(stitch,lidar_b,index);

    return 0;
}


void PclPointStitchA(pcl::PointCloud<PointXYZSTITCH>::Ptr& stitch,const pcl::PointCloud<PointXYZA>::Ptr& pcl_a,int &index) {
    for(int i=0; i < pcl_a->points.size(); i++, index++)
    {
      stitch->points[index].x = pcl_a->points[i].x;
      stitch->points[index].y = pcl_a->points[i].y;
      stitch->points[index].z = pcl_a->points[i].z;
      stitch->points[index].intensity = pcl_a->points[i].intensity;
    }
}

void PclPointStitchB(pcl::PointCloud<PointXYZSTITCH>::Ptr& stitch,const pcl::PointCloud<PointXYZB>::Ptr& pcl_b,int &index) {
    for(int i=0; i < pcl_b->points.size(); i++, index++)
    {
      stitch->points[index].x = pcl_b->points[i].x;
      stitch->points[index].y = pcl_b->points[i].y;
      stitch->points[index].z = pcl_b->points[i].z;
      stitch->points[index].intensity = pcl_b->points[i].intensity;
    }
}