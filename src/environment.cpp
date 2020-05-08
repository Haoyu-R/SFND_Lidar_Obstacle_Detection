/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


//void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
//{
//    // ----------------------------------------------------
//    // -----Open 3D viewer and display simple highway -----
//    // ----------------------------------------------------
//    
//    // RENDER OPTIONS
//    bool renderScene = false;
//    std::vector<Car> cars = initHighway(renderScene, viewer);
//    
//    // TODO:: Create lidar sensor 
//    Lidar *lidar = new Lidar(cars, 0);
//    auto point_clouds = lidar->scan();
//
//    //renderRays(viewer, lidar->position, point_clouds);
//    //renderPointCloud(viewer, point_clouds, "test", Color(1, 1, 1));
//    
//    // TODO:: Create point processor
//
//    ProcessPointClouds<pcl::PointXYZ>* point_processor = new ProcessPointClouds<pcl::PointXYZ>();
//    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentation = point_processor->SegmentPlane(point_clouds, 50, 0.2);
//
//    /*renderPointCloud(viewer, segmentation.first, "plane", Color(0, 1, 0));
//    renderPointCloud(viewer, segmentation.second, "obstacle", Color(1, 0, 0));
//    */
//
//    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = point_processor->Clustering(segmentation.first, 1.0, 3, 30);
//
//    int clusterID = 0;
//    std::vector<Color> colors = { Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1) };
//
//    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
//        std::cout << "Cluster Size";
//        point_processor->numPoints(cluster);
//        Box box = point_processor->BoundingBox(cluster);
//        renderBox(viewer, box, clusterID);
//        renderPointCloud(viewer, cluster, "ObstCloud" + std::to_string(clusterID), colors[clusterID]);
//
//        clusterID++;
//       
//    }
//  
//}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* point_processor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_frame)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    cloud = point_processor->FilterCloud(cloud_frame, 0.3f, Eigen::Vector4f (-10, -6, -3, 1), Eigen::Vector4f (25, 6, 3, 1));
    
    //renderPointCloud(viewer, cloud, "pointCloud");

    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentation = point_processor->SegmentPlane(cloud, 25, 0.3);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = point_processor->Clustering(segmentation.first, 0.53, 10);

    renderPointCloud(viewer, segmentation.second, "plane", Color(0, 1, 0));


    int boxID = 0;
    for (auto cluster : clusters) {
        std::cout << "Cluster Size: ";
        point_processor->numPoints(cluster);
        Box box = point_processor->BoundingBox(cluster);
        renderBox(viewer, box, boxID);
        renderPointCloud(viewer, cluster, std::to_string(boxID), Color(1, 0, 0));
        boxID++;
    }

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* point_processor = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_frame(new pcl::PointCloud<pcl::PointXYZI>());

    std::vector<boost::filesystem::path> stream = point_processor->streamPcd("../src/sensors/data/pcd/data_1");

    //std::vector<boost::filesystem::path> stream = point_processor->streamPcd("C:\\Users\\arhyr\\Desktop\\sensor fusion\\SFND_Lidar_Obstacle_Detection\\src\\sensors\\data\\pcd\\data_1");
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = point_processor->loadPcd("C:\\Users\\arhyr\\Desktop\\sensor fusion\\SFND_Lidar_Obstacle_Detection\\src\\sensors\\data\\pcd\\data_1\\0000000000.pcd");
    auto streamIterator = stream.begin();


    while (!viewer->wasStopped ())
    {
        // clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //load pcd and run detection
        point_frame = point_processor->loadPcd((*streamIterator).string());

        cityBlock(viewer, point_processor, point_frame);

        streamIterator++;

        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        // control frame rate
        viewer->spinOnce ();
    } 
}