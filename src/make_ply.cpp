#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "./../include/make_ply.hpp"

ROStoPCL::ROStoPCL() : 
    cloud (new pcl::PointCloud<pcl::PointXYZ>()),
    filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>()),
    R (Eigen::Matrix4d::Identity()),
    filename ("/mnt/container-data/model_for_RT/trimed_3d_model.ply"),
    save_name ("/mnt/container-data/model_for_RT/filtered_3D_model.ply")
{
    rotevec = new GetRotationVector();
    edit = new EditCloud();
}

ROStoPCL::~ROStoPCL()
{
    delete rotevec; 
    delete edit;
}

void ROStoPCL::read_cloud()
{
    pcl::io::loadPLYFile(filename, *cloud);
}

void ROStoPCL::savePointcloud()
{
    pcl::io::savePLYFileASCII(save_name, *filtered_cloud);
}

void ROStoPCL::transformPointCloud() {

    pcl::copyPointCloud(*cloud, *(edit->cloud));
    edit->filter();

    pcl::copyPointCloud(*(edit->cloud), *filtered_cloud);

    std::cout << "filtered num = " << filtered_cloud->size() << std::endl;

    pcl::transformPointCloud(*cloud, *cloud, R); 
    pcl::transformPointCloud(*filtered_cloud, *filtered_cloud, R); 
}

void ROStoPCL::quaternion_to_euler() {

    rotevec->tpclZ =  0.0;
    rotevec->tpclY = 0.0;
    rotevec->tpclX = 0.0;

    rotevec->roll  = -20.0*(M_PI/180);
    rotevec->pitch = 0.0;
    rotevec->yaw   = 0.0;

    rotevec->transformPointCloud();
    R = rotevec->R;
}

void ROStoPCL::visualize_cloud()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Matched Viewer"));
	viewer->setBackgroundColor(255, 255, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_1(cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, handler_1, "cloud 1");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_2(filtered_cloud, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZ>(filtered_cloud, handler_2, "cloud 2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "filtered_cloud");
	
    viewer->addCoordinateSystem();

	//The position of the camera
	while (!viewer->wasStopped())
    {
		viewer->spinOnce(100);
	}
}

void ROStoPCL::run()
{
    read_cloud();
    quaternion_to_euler();
    transformPointCloud();
    savePointcloud();
    visualize_cloud();
}

int main(int argc, char *argv[])
{

    ROStoPCL *get_pcl;
    get_pcl = new ROStoPCL();
    get_pcl->run();

    delete get_pcl; 
   
    return 0;
}
