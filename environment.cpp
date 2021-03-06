/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

//#include "sensors/lidar.h"
//#include "render/render.h"
//#include "processPointClouds.h"
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

/*
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);
    //renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    //renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    //ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    /* manual clusting
	  std::vector<std::vector<float>> points;

    KdTree* tree = new KdTree;
    for (int i=0; i<segmentCloud.first->points.size(); i++)
    {
      float x = segmentCloud.first->points[i].x;
		  float y = segmentCloud.first->points[i].y;
		  float z = segmentCloud.first->points[i].z;
      std::vector<float> point {x, y, z};
    	tree->insert(point, i);
    }
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.euclideanClusterI(segmentCloud.first, tree, 2.5, 3, 40);
    */ /* end manual clustering

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 2.5, 3, 40);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
    //if(render_clusters)
    /*{
        std::cout << "cluster size ";
      pointProcessor.numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
    */ //}
    //if(render_box)
    /*{ /*
      Box box = pointProcessor.BoundingBox(cluster);
      renderBox(viewer,box,clusterId);
    //} */
      //++clusterId;
    //} 
    //renderPointCloud(viewer, segmentCloud, second, "planeCloud");
//} 
// end simple highway

//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  //renderPointCloud(viewer,inputCloud,"inputCloud");

  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.4, Eigen::Vector4f (-10, -6, -5000, 1), Eigen::Vector4f ( 40.0, 7.0, 3000.0, 1));
  //renderPointCloud(viewer,filterCloud,"filterCloud");

  //segmentation
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.1);
  //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
  //renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

  //clustering
  //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 30, 4000);
    // manual clusting
	  std::vector<std::vector<float>> points(segmentCloud.first->points.size());
    KdTree* tree = new KdTree;
    for (int i=0; i<segmentCloud.first->points.size(); i++)
    {
      float x = segmentCloud.first->points[i].x;
		  float y = segmentCloud.first->points[i].y;
		  float z = segmentCloud.first->points[i].z;
      std::vector<float> point {x, y, z};
      points[i]= point;
    	tree->insert(point, i);
    }
    //std::cout<< points[268][2]<<std::endl;
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanClusterI(segmentCloud.first, tree, 0.5, 30, 200);
    std::vector<std::vector<int>> clusters = pointProcessorI->euclideanClusterI(points, tree, 0.5, 30, 200);
    //std::cout<<clusters[1][25]<< std::endl;
    // //end manual clustering

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    //for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    for(std::vector<int> cluster : clusters)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
  		for(int indice: cluster)
  		  clusterCloud->points.push_back(segmentCloud.first->points[indice]);
        //clusterCloud->points.push_back(pcl::PointXYZI (points[indice][0], points[indice][1], points[indice][2], 0));
      std::cout << "clusterCloud size ";
      pointProcessorI->numPoints(clusterCloud);
      renderPointCloud(viewer,clusterCloud,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

      Box box = pointProcessorI->BoundingBox(clusterCloud);
      renderBox(viewer,box,clusterId);

      ++clusterId;
    }
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
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
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    //simpleHighway(viewer);
    //cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        
        viewer->spinOnce ();
    } 
}