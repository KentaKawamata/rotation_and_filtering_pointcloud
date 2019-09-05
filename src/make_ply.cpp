#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include "./../include/make_ply.hpp"

ROStoPCL::ROStoPCL() : 
    cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>()),
    R (Eigen::Matrix4d::Identity()),
    cloud_frame ("/camera/depth/color/points"),
    count (0),
    filename ("/root/datas/model_for_RT/3d_model.ply")
{
    rotevec = new GetRotationVector();
    edit = new EditCloud();
}

ROStoPCL::~ROStoPCL()
{
    delete rotevec; 
    delete edit;
}

void ROStoPCL::savePointcloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPLYFile(filename, *total_cloud);

    pcl::io::savePLYFileASCII(filename, *cloud_pcl);
}

void ROStoPCL::transformPointCloud() {

    pcl::copyPointCloud(*cloud_pcl, *(edit->cloud));
    edit->filter();
    pcl::copyPointCloud(*(edit->cloud), *cloud_pcl);
    pcl::transformPointCloud(*cloud_pcl, *cloud_pcl, R); 

    savePointcloud();
}

void ROStoPCL::quaternion_to_euler() {

    rotevec->tpclZ =  0.0;
    rotevec->tpclY = 0.0;
    rotevec->tpclX = 0.0;

    rotevec->roll  = 20*(M_PI/180);
    rotevec->pitch = 0.0;
    rotevec->yaw   = 0.0;

    rotevec->transformPointCloud();
    R = rotevec->R;
}

void ROStoPCL::run()
{
    quaternion_to_euler();
    transformPointCloud();
}

int main(int argc, char *argv[])
{

    ROStoPCL *get_pcl;
    get_pcl = new ROStoPCL();
    get_pcl->run();

    delete get_pcl; 
   
    return 0;
}
