// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
    // Create the filtering object
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

// function seperate clouds just for pcl Ransac implementation
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud,planeCloud);
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}

// helping function just for manual Ransac implementation
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));

		float x1, y1, x2, y2, x3, y3, z1, z2, z3;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
    
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;

		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;


		float i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		float D = -(i*x1+j*y1+k*z1);

		for(int index = 0; index < cloud->points.size(); index ++)
		{
			if (inliers.count(index)>0)
				continue;
			auto point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float d = fabs(i*x4+j*y4+k*z4+D)/sqrt(i*i+j*j+k*k);

			if(d <= distanceThreshold)
				inliers.insert(index);
		}
		if(inliers.size()>inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	return inliersResult;

}
// end helping functions manual Ransac implementation

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;

    /* pcl Ransac implementation
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    //Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    */ //end pcl Ransac implementation

    // manual Ransac implementation
	std::unordered_set<int> inliers = RansacPlane(cloud, maxIterations, distanceThreshold);

	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		auto point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult (cloudOutliers,cloudInliers);
    // //end manual Ransac implementation
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    return segResult;
}
/* start pcl clustering
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloudCluster->points.push_back (cloud->points[index]);

            cloudCluster->width = cloudCluster ->points.size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;

            clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
*/ // end pcl clustering

//functions for manual Clustering with kd tree
template<typename PointT>
//void ProcessPointClouds<PointT>::clusterHelperI(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr clusterCloud, std::vector<bool> processed, KdTree* tree, float distanceTol)
//void ProcessPointClouds<PointT>::clusterHelperI(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool> processed, KdTree* tree, float distanceTol)
void ProcessPointClouds<PointT>::clusterHelperI(const int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool> processed, KdTree* tree, const float distanceTol)
{
	processed[indice] = true; 
	//clusterCloud->points.push_back(cloud->points[indice]);
    cluster.push_back(indice);
    /*
    float x = cloud->points[indice].x;
	float y = cloud->points[indice].y;
	float z = cloud->points[indice].z;
    std::vector<float> target {x, y, z};
    */ //std::cout << x << std::endl;
    
	//std::vector<int> nearest = tree->search(cloud->points[indice], distanceTol);
    //std::vector<int> nearest = tree->search(target, distanceTol);
    std::vector<int> nearest = tree->search(points[indice], distanceTol);
    //std::cout << nearest[0] << std::endl;

	for(int id : nearest)
	{
		if(!processed[id])
			clusterHelperI(id, points, cluster, processed, tree, distanceTol);
	}
}

template<typename PointT>
//std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanClusterI(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize)
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanClusterI(const std::vector<std::vector<float>> points, KdTree* tree, const float distanceTol, const int minSize, const int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();
    
    //std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<int>> clusters;

    /* Versuch mit const matrix als input
    std::vector<std::vector<float>> points;

    for (int i=0; i<cloud->points.size(); i++)
    {
        float x = cloud->points[i].x;
	    float y = cloud->points[i].y;
	    float z = cloud->points[i].z;
        std::vector<float> point {x, y, z};
        points.push_back(point);
    }
     */   //std::cout << points[0][0] << std::endl;
    // end Versuch

	std::vector<bool> processed(points.size(), false);

	int i = 0;
	while(i < points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}
	
		//typename pcl::PointCloud<PointT>::Ptr clusterCloud (new pcl::PointCloud<PointT>());
        std::vector<int> cluster;
		//clusterHelperI(i, cloud, cluster, processed, tree, distanceTol);
        clusterHelperI(i, points, cluster, processed, tree, distanceTol);
        //std::cout << cluster[0] << std::endl;
        //if (clusterCloud->points.size() > minSize && clusterCloud->points.size() < maxSize)
        if (cluster.size() > minSize && cluster.size() < maxSize)
        {
            //typename pcl::PointCloud<PointT>::Ptr clusterCloud (new pcl::PointCloud<PointT>());
  		    //for(int indice: cluster)
  		        //clusterCloud->points.push_back(cloud->points[indice]);
            clusters.push_back(cluster);
        }
        else
        {
            cluster.clear();
        }
		i++;
	}
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

	return clusters;
}

 //end eucllidean Cluster functions

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