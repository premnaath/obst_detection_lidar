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

    float filterRes = 0.15f;
    Eigen::Vector4f minPoint(-10.0, -6.0, -2.0, 1.0);
    Eigen::Vector4f maxPoint(32.0, 6.5, 5.0, 1.0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = point_processorI->FilterCloud(inputCloud, filterRes, minPoint, maxPoint);
    //renderPointCloud(viewer, filtered_cloud, "InputCloud");

    // Segment road and obstacle
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented_cloud = point_processorI->SegmentPlane(filtered_cloud, 100, 0.2);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented_cloud = point_processorI->SegmentPlaneRansac(filtered_cloud, 25, 0.2);
    renderPointCloud(viewer, segmented_cloud.first, "Obstacle cloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmented_cloud.second, "Road cloud", Color(0, 1, 0));

    /*
    // Cluster things
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = point_processorI->Clustering(segmented_cloud.first, 0.6, 4, 3000);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1), Color(1, 1, 0), Color(0, 1, 1), Color(1, 1, 1),
                                 Color(0.5, 0, 0), Color(0, 0.5, 0), Color(0, 0, 0.5), Color(0.5, 0.5, 0), Color(0, 0.5, 0.5)};

    pcl::PointXYZI centroid;
    pcl::computeCentroid(*segmented_cloud.second, centroid);
    std::cout << "Centroid : " << centroid.z << std::endl;

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        point_processorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);

        Box box = point_processorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        //BoxQ boxq = point_processorI->BoundingBoxQ(cluster);
        //renderBox(viewer, boxq, clusterId, colors[0]);

        ++clusterId;
    }*/
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    // renderRays(viewer, lidar->position, cloud);
    // renderPointCloud(viewer, cloud, "Point cloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> *point_processor = new ProcessPointClouds<pcl::PointXYZ>();

    /*std::stack<ProcessPointClouds<pcl::PointXYZ>> point_processor_stack;
    point_processor_stack.push(ProcessPointClouds<pcl::PointXYZ>());*/

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented_cloud = point_processor->SegmentPlane(cloud, 100, 0.2);

    // renderPointCloud(viewer, segmented_cloud.first, "Obstacle cloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmented_cloud.second, "Road cloud", Color(0, 1, 0));

    // Cluster things
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = point_processor->Clustering(segmented_cloud.first, 1.5, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        point_processor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);

        Box box = point_processor->BoundingBox(cluster);
        //renderBox(viewer, box, clusterId);

        BoxQ boxq = point_processor->BoundingBoxQ(cluster);
        renderBox(viewer, boxq, clusterId, colors[2]);

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