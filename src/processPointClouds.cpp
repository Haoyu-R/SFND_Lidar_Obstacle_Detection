// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "quiz\cluster\kdtree.h"
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud);

    pcl::CropBox<PointT> cor(true);
    cor.setInputCloud(cloud);
    cor.setMax(maxPoint);
    cor.setMin(minPoint);
    cor.filter(*cloud);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMax(Eigen::Vector4f (2.7, 1.5, 0, 1));
    roof.setMin(Eigen::Vector4f (-2, -1.5, -3, 1));
    roof.setInputCloud(cloud);
    roof.filter(indices);

    for (int index : indices) {
        inliers->indices.push_back(index);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    // Push inlier to plane cloud by indexing
    for (int index : inliers->indices)
        planeCloud->points.emplace_back(cloud->points[index]);
    
    //seperate obst cloud
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
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

    int max_num = cloud->points.size();

    PointT point1;
    PointT point2;
    PointT point3;

    double i = 0, j = 0, k = 0, D = 0;
    double dist = 0;
    double denominator = 1;

    while (maxIterations--) {
        std::unordered_set<int> inliersTemp;
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
            if (inliersTemp.count(index) > 0)
                continue;
            PointT point = cloud->points[index];
            dist = fabs(i * point.x + j * point.y + k * point.z + D) / denominator;
            if (dist < distanceThreshold)
                inliersTemp.insert(index);
        }

        if (inliersTemp.size() > inliersResult.size()) {
            inliersResult = std::move(inliersTemp);
        }

    }
    
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (auto index : inliersResult) {
        inliers->indices.push_back(index);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


inline void proximity(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int index, std::vector<bool>& flags, KdTree<pcl::PointXYZI>* tree, pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster, float distanceTol) {

    flags[index] = true;
    cluster->points.push_back(cloud->points[index]);

    std::vector<int> neighbour = tree->search(cloud->points[index], distanceTol);

    //std::cout << "neighbour number: " << neighbour.size() << std::endl;

    for (int i : neighbour) {
        if (!flags[i])
            proximity(cloud, i, flags, tree, cluster, distanceTol);
    }

}

inline std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> euclideanCluster(const typename pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, KdTree<pcl::PointXYZI>* tree, float distanceTol, int minSize)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    
    int point_sum = cloud->points.size();

    std::vector<bool> flags(point_sum, false);

    for (int index = 0; index < point_sum; index++) {
        if (!flags[index]) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
            proximity(cloud, index, flags, tree, cluster, distanceTol);
            if(cluster->points.size() > minSize)
                clusters.push_back(cluster);
        }
    }
    return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize)
{
    //// TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //typename pcl::search::KdTree<PointT>::Ptr  tree(new pcl::search::KdTree<PointT>());
    //tree->setInputCloud(cloud);

    //std::vector<pcl::PointIndices> clusterIndices;
    //pcl::EuclideanClusterExtraction<PointT> ec;
    //ec.setClusterTolerance(clusterTolerance);
    //ec.setMinClusterSize(minSize);
    //ec.setMaxClusterSize(maxSize);
    //ec.setSearchMethod(tree);
    //ec.setInputCloud(cloud);
    //ec.extract(clusterIndices);

    //for (pcl::PointIndices getIndices : clusterIndices) {
    //    typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>());
    //    
    //    for (int index : getIndices.indices) {
    //        cloudCluster->points.push_back(cloud->points[index]);
    //    }

    //    cloudCluster->width = cloudCluster->points.size();
    //    cloudCluster->height = 1;
    //    cloudCluster->is_dense = true;

    //    clusters.push_back(cloudCluster);
    //}

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    KdTree<PointT>* tree(new KdTree<PointT>());

    for (int i = 0; i < cloud->points.size(); i++) {
        tree->insert(cloud->points[i], i);
    }

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    clusters = euclideanCluster(cloud, tree, clusterTolerance, minSize);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}