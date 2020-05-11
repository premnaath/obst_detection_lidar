// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

    // Voxel grid filtering
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(filterRes, filterRes, filterRes);
    vox.filter(*cloud_filtered);

    // ROI filtering
    pcl::CropBox<PointT> cb;
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    //cb.setNegative(true);
    cb.setInputCloud(cloud_filtered);
    cb.filter(*cloud_filtered);

    // Roof reflection filtering
    pcl::CropBox<PointT> cb_roof;
    cb_roof.setMin(Eigen::Vector4f(-3, -1.2, -1, 1));
    cb_roof.setMax(Eigen::Vector4f(3, 1.2, 0.5, 1));
    cb_roof.setNegative(true);
    cb_roof.setInputCloud(cloud_filtered);
    cb_roof.filter(*cloud_filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;
}

template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Init return var
    std::unordered_set<int> inliersResult;

    // For max iterations
    while (maxIterations--)
    {
        // Randomly sample subset and fit line
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
        {
            inliers.insert(rand() % cloud->points.size());
        }

        //Fit a plane with 3 points
        float x1, y1, z1, x2, y2, z2, x3, y3, z3, A, B, C, D;

        std::unordered_set<int>::iterator iter = inliers.begin();
        x1 = cloud->points[*iter].x;
        y1 = cloud->points[*iter].y;
        z1 = cloud->points[*iter].z;
        iter++;
        x2 = cloud->points[*iter].x;
        y2 = cloud->points[*iter].y;
        z2 = cloud->points[*iter].z;
        iter++;
        x3 = cloud->points[*iter].x;
        y3 = cloud->points[*iter].y;
        z3 = cloud->points[*iter].z;

        A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        D = -(A * x1 + B * y1 + C * z1);

        // Measure distance between every point and fitted line
        for (int i = 0; i < cloud->points.size(); i++)
        {
            if (inliers.count(i) > 0)
                continue;

            float x = cloud->points[i].x;
            float y = cloud->points[i].y;
            float z = cloud->points[i].z;

            float distance = fabs(x * A + y * B + C * z + D) / sqrt(A * A + B * B + C * C);

            if (distance <= distanceTol)
            {
                inliers.insert(i);
            }
        }

        // Return indicies of inliers from fitted line with most inliers
        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }

    return inliersResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Find the plane of best fit - RANSAC 3D
    std::unordered_set<int> inliers = RansacPlane(cloud, maxIterations, distanceThreshold);

    // Point cloud to separate Road and Obstacle
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    // Separate
    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    // Pair segmented data
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
    segResult.first = cloudOutliers;
    segResult.second = cloudInliers;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Separating segmented data took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

// Find the proximity points in the KD-tree within a given distance, min and max cluster size
// This function finds the complete cluster with recursive calls
template <typename PointT>
void ProcessPointClouds<PointT>::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree *tree, float distanceTol, std::vector<bool> &processed, int index, pcl::PointIndices &cluster, int maxSize)
{
    if (processed[index] && cloud->points.size() < maxSize)
    {
        return;
    }

    processed[index] = true;
    cluster.indices.push_back(index);

    pcl::PointXYZI point = cloud->points[index];

    std::vector<int> nearby_points = tree->search(point, distanceTol);

    for (int i = 1; i < nearby_points.size(); i++)
    {
        if (!processed[nearby_points[i]])
        {
            proximity(cloud, tree, distanceTol, processed, nearby_points[i], cluster, maxSize);
        }
    }
}

// Cluster the segmented data using a specified euclidean distance
template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree *tree, float distanceTol, int minSize, int maxsize)
{
    // Init
    std::vector<std::vector<int>> clusters;
    std::size_t cloud_size = cloud->points.size();
    std::vector<bool> processed(cloud_size, false);

    // Cle
    for (int i = 0; i < cloud_size; i++)
    {
        if (processed[i])
        {
            continue;
        }
        
        // Call Proximity function with a referenced cluster
        pcl::PointIndices cluster;
        proximity(cloud, tree, distanceTol, processed, i, cluster, maxsize);
        
        // Push back identified clusters of required size
        std::size_t cluster_size = cluster.indices.size();

        if (cluster_size > minSize && cluster_size < maxsize)
            clusters.push_back(cluster.indices);
    }

    return clusters;
}

// Create a bounding box to the given clustered data
template <typename PointT>
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}