#ifndef MAKE_PLY_H
#define MAKE_PLY_H

#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/LU>

#include "./../include/getRotationVector.hpp"
#include "./../include/editCloud.hpp"

class ROStoPCL {

private:

    std::string  filename;
    std::string  save_name;

    std::string lis_header_id;
    std::string lis_child_id;

    std::string cloud_frame;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;

    Eigen::Matrix3d R3d;
    Eigen::Matrix4d R;

    void savePointcloud();
    void transformPointCloud();
    void quaternion_to_euler(); 
    void read_cloud();
    void visualize_cloud();

public:

    ROStoPCL();
    ~ROStoPCL();
    void run();

    GetRotationVector *rotevec;
    EditCloud *edit; 

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif //MAKE_PLY_H