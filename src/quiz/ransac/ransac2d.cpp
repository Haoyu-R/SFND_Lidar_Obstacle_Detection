/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    int max_num = cloud->points.size();
    int point_num_1 = 0;
    int point_num_2 = 0;

    pcl::PointXYZ point1;
    pcl::PointXYZ point2;

    double A = 0, B = 0, C = 0;
    int fit = 0;
    double dist = 0;

    double best_A = 0, best_B = 0, best_C = 0;
    int best_fit = 0;
    double denominator = 1;

    while (maxIterations--) {
        point_num_1 = rand() % max_num;
        point_num_2 = rand() % max_num;

        while (point_num_2 == point_num_1) {
            point_num_2 = rand() % max_num;
        }

        point1 = cloud->points[point_num_1];
        point2 = cloud->points[point_num_2];

        A = point1.y - point2.y;
        B = point2.x - point1.x;
        C = point1.x * point2.y - point2.x * point1.y;

        denominator = sqrt(A * A + B * B);

        for (auto point : cloud->points) {

            dist = abs(A * point.x + B * point.y + C) / denominator;
            if (dist < distanceTol)
                fit++;
        }
        if (fit > best_fit) {
            best_fit = fit;
            best_A = A;
            best_B = B;
            best_C = C;
        }

        fit = 0;
    }

    denominator = sqrt(best_A * best_A + best_B * best_B);

    for (int index = 0; index < cloud->points.size(); index++) {
        pcl::PointXYZ point = cloud->points[index];
        dist = abs(best_A * point.x + best_B * point.y + best_C) / denominator;
        if (dist < distanceTol)
            inliersResult.insert(index);
    }
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
    
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    std::unordered_set<int> inliersTemp;
    int max_num = cloud->points.size();

    pcl::PointXYZ point1;
    pcl::PointXYZ point2;
    pcl::PointXYZ point3;

    double i = 0, j = 0, k = 0, D = 0;
    double dist = 0;
    double denominator = 1;

    while (maxIterations--) {

        while (inliersTemp.size() < 3) {
            inliersTemp.insert(rand() % max_num);
        }

        point1 = cloud->points[inliersTemp[0]];
        point2 = cloud->points[inliersTemp[1]];
        point3 = cloud->points[inliersTemp[2]];


        i = (point2.y - point1.y)*(point3.z-point1.z) - (point2.z-point1.z)*(point3.y-point1.y);
        j = (point2.z - point1.z)*(point3.x-point1.x) - (point2.x-point1.x)* (point3.z - point1.z);
        k = (point2.x - point1.x)* (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
        D = -(i * point1.x + j * point1.y + k * point1.z);

        denominator = sqrt(i * i + j * j + k*k);

        for (int index = 0; index < max_num; index++) {
            if (inliersTemp.count(index))
                continue;
            pcl::PointXYZ point = cloud->points[index];
            dist = abs(i * point.x + j * point.y + k * point.z + D) / denominator;
            if (dist < distanceTol)
                inliersTemp.insert(index);
        }

        if (inliersTemp.size() > inliersResult.size()) {
            inliersResult = inliersTemp;
        }

    }
    return inliersResult;

}
int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
