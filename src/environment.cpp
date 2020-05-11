/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI>* point_processorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    //ProcessPointClouds<pcl::PointXYZI> *point_processorI(new ProcessPointClouds<pcl::PointXYZI>());
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = point_processorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    float filterRes = 0.25f;
    Eigen::Vector4f minPoint(-10.0, -6.0, -2.0, 1.0);
    Eigen::Vector4f maxPoint(32.0, 7.0, 5.0, 1.0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = point_processorI->FilterCloud(inputCloud, filterRes, minPoint, maxPoint);
    //renderPointCloud(viewer, filtered_cloud, "InputCloud");

    // Segment road and obstacle
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented_cloud = point_processorI->SegmentPlane(filtered_cloud, 100, 0.2);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented_cloud = point_processorI->SegmentPlaneRansac(filtered_cloud, 50, 0.2);
    //renderPointCloud(viewer, segmented_cloud.first, "Obstacle cloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmented_cloud.second, "Road cloud", Color(0.5, 0.5, 0.5));

    // Cluster with KD-Tree
    KdTree *tree = new KdTree;

	for (int i = 0; i < segmented_cloud.first->points.size(); i++)
		tree->insert(segmented_cloud.first->points[i], i);

    // Time segmentation process
	auto startTime = std::chrono::steady_clock::now();
	//
	//std::vector<std::vector<int>> clusters = point_processorI->euclideanCluster(segmented_cloud.first, tree, 0.45, 10, 1000);
    //std::vector<std::vector<int>> clusters = point_processorI->euclideanCluster(segmented_cloud.first, tree, 0.45, 10, 550);
    std::vector<std::vector<int>> clusters = point_processorI->euclideanCluster(segmented_cloud.first, tree, 0.45, 8, 550);
	//
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

    // Separate clusters

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;
    
    int clusterId = 0;
	for (std::vector<int> cluster : clusters)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
		for (int indice : cluster)
        {
                clusterCloud->points.push_back(segmented_cloud.first->points[indice]);
        }
			
        
        cloudClusters.push_back(clusterCloud);
		++clusterId;
	}

	std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1), Color(1, 1, 0), Color(0, 1, 1), Color(1, 1, 1),
                                 Color(0.5, 0, 0), Color(0, 0.5, 0), Color(0, 0, 0.5), Color(0.5, 0.5, 0), Color(0, 0.5, 0.5)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        point_processorI->numPoints(cluster);
        int color_id = clusterId % colors.size();
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[color_id]);

        Box box = point_processorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    // City block functions
    ProcessPointClouds<pcl::PointXYZI> *point_processorI(new ProcessPointClouds<pcl::PointXYZI>());
    std::vector<boost::filesystem::path> stream = point_processorI->streamPcd("../src/sensors/data/pcd/data_1");
    std::vector<boost::filesystem::path>::iterator iter_stream = stream.begin();
    
    while (!viewer->wasStopped())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load ad call cityblock functino
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = point_processorI->loadPcd((*iter_stream).string());
        cityBlock(viewer, point_processorI, inputCloud);

        iter_stream++;

        if (iter_stream == stream.end())
        {
            iter_stream = stream.begin();
        }

        viewer->spinOnce();
    }
}