/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}


void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%2==0)
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}

void proximity(const std::vector<std::vector<float>>& points, const int & index, std::vector<bool>& flags, KdTree * tree, std::vector<int> &cluster, float distanceTol) {

    flags[index] = true;
    cluster.push_back(index);

    std::vector<int> neighbour = tree->search(points[index], distanceTol);
    for (int i : neighbour) {
        if (!flags[i])
            proximity(points, i, flags, tree, cluster, distanceTol);
    }

}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
    
    int point_sum = points.size();

    std::vector<bool> flags(point_sum, false);

    for (int index = 0; index < point_sum; index++) {
        if (!flags[index]) {

            std::vector<int> cluster;
            proximity(points, index, flags, tree, cluster, distanceTol);
            clusters.push_back(cluster);           
        }
    }
	return clusters;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //// TODO:: Fill in this function to find inliers for the cloud.

    //// Instantiat 2 pointers
    //pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); // inlier point indice
    //pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients()); //coefficients of modell
    //
    //pcl::SACSegmentation<PointT> seg;

    //// optional
    //seg.setOptimizeCoefficients(true);

    ////set segmentation parameter
    //seg.setModelType(pcl::SACMODEL_PLANE);
    //seg.setMethodType(pcl::SAC_RANSAC);
    //seg.setMaxIterations(maxIterations);
    //seg.setDistanceThreshold(distanceThreshold);

    //// input data and 
    //seg.setInputCloud(cloud);
    //seg.segment(*inliers, *coefficients);


    // If no model found fit the data
    /*if (inliers->indices.size() == 0) {
        std::cout << "Cannot find a plane" << std::endl;
    }*/

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    std::unordered_set<int> inliersTemp;
    int max_num = cloud->points.size();

    PointT point1;
    PointT point2;
    PointT point3;

    double i = 0, j = 0, k = 0, D = 0;
    double dist = 0;
    double denominator = 1;

    while (maxIterations--) {

        // randomly pick indices of three points
        while (inliersTemp.size() < 3) {
            inliersTemp.insert(rand() % max_num);
        }

        auto iterator = inliersTemp.begin();
        point1 = cloud->points[(*iterator++)];
        point2 = cloud->points[(*iterator++)];
        point3 = cloud->points[(*iterator)];


        i = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
        j = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
        k = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
        D = -(i * point1.x + j * point1.y + k * point1.z);

        denominator = sqrt(i * i + j * j + k * k);

        for (int index = 0; index < max_num; index++) {
            if (inliersTemp.count(index))
                continue;
            PointT point = cloud->points[index];
            dist = abs(i * point.x + j * point.y + k * point.z + D) / denominator;
            if (dist < distanceThreshold)
                inliersTemp.insert(index);
        }

        if (inliersTemp.size() > inliersResult.size()) {
            inliersResult = inliersTemp;
        }

    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (auto index : inliersResult) {
        inliers->indices.push_back(index);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


void proximity(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int index, std::vector<bool>& flags, typename KdTree<pcl::PointXYZI>* tree, typename pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster, float distanceTol) {

    flags[index] = true;
    cluster->push_back(cloud->points[index]);

    std::vector<int> neighbour = tree->search(cloud->points[index], distanceTol);

    for (int i : neighbour) {
        if (!flags[i])
            proximity(cloud, i, flags, tree, cluster, distanceTol);
    }

}

std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> euclideanCluster(const typename pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, typename KdTree<pcl::PointXYZI>* tree, float distanceTol, int minSize)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

    int point_sum = cloud->points.size();

    std::vector<bool> flags(point_sum, false);

    for (int index = 0; index < point_sum; index++) {
        if (!flags[index]) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster;
            proximity(cloud, index, flags, tree, cluster, distanceTol);
            if (cluster->points.size() > minSize)
                clusters.push_back(cluster);
        }
    }
    return clusters;
}


int main ()
{

	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min =   0;
  	window.z_max =   0;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
	std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

	KdTree* tree = new KdTree;
  
    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i); 

  	int it = 0;
  	render2DTree(tree->root,viewer,window, it);
  
  	std::cout << "Test Search" << std::endl;
  	std::vector<int> nearby = tree->search({-6,7},3.0);
  	for(int index : nearby)
      std::cout << index << ",";
  	std::cout << std::endl;

  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	//
  	std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0);
  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],0));
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer,cloud,"data");
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
