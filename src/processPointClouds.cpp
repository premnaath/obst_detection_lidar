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
    cb_roof.setMin(Eigen::Vector4f(-3, -2, -3, 1));
    cb_roof.setMax(Eigen::Vector4f(3, 2, 0.5, 1));
    cb_roof.setNegative(true);
    cb_roof.setInputCloud(cloud_filtered);
    cb_roof.filter(*cloud_filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;
}
/*
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterGhost(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // This function removes ghost objects detected below the road
    // Input: 
}
*/

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr road_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());

    for (int index : inliers->indices)
    {
        road_cloud->points.push_back(cloud->points[index]);
    }

    // Separate the point clouds with extract object
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, road_cloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process1
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // TODO:: Fill in this function to find inliers for the cloud.
    // Define necessary parameters
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // Define the segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segmentation
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

//template<typename PointT>
//std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
    srand(time(NULL));

    // TODO: Fill in this function
    std::size_t prev_inliers = 0;

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
        // If distance is smaller than threshold count it as inlier
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_road(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_obst(new pcl::PointCloud<pcl::PointXYZI>());

        for (int i = 0; i < cloud->points.size(); i++)
        {
            if (inliers.count(i) > 0)
                continue;

            float x = cloud->points[i].x;
            float y = cloud->points[i].y;
            float z = cloud->points[i].z;

            float distance = fabs(x * A + y * B + C * z + D) / sqrt(A * A + B * B + C * C);
            // float distance = std::abs(x * (y1 - y2) + y * (x2 - x1) + (x1 * y2 - x2 * y1)) / std::sqrt((y1 - y2) * (y1 - y2) + (x2 - x1) * (x2 - x1));

            if (distance <= distanceTol)
            {
                inliers.insert(i);
                temp_cloud_road->points.push_back(cloud->points[i]);
            }
            else
            {
                temp_cloud_obst->points.push_back(cloud->points[i]);
            }
        }

        // Return indicies of inliers from fitted line with most inliers
        if (inliers.size() > inliersResult.size()) //  &&  inliers.size()/cloud->points.size() > 0.9
        {
            inliersResult = inliers;
            //std::cout << "Inliers size : " << inliers.size() << " Total size : " << cloud->points.size() << std::endl;
            
            std::cout << "Iterations" << maxIterations << std::endl;
            segResult.first = temp_cloud_obst;
            segResult.second = temp_cloud_road;
        }
    }

    return segResult; // inliersResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    // std::unordered_set<int> inliers = RansacPlane(cloud, maxIterations, distanceThreshold);

    // //std::unordered_set<int> inliers;
    // //RansacPlane(cloud, maxIterations, distanceThreshold, inliers);

    // typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    // typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    // for (int index = 0; index < cloud->points.size(); index++)
    // {
    //     //pcl::PointXYZ point = cloud->points[index];
    //     PointT point = cloud->points[index];
    //     if (inliers.count(index))
    //         cloudInliers->points.push_back(point);
    //     else
    //         cloudOutliers->points.push_back(point);
    // }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = RansacPlane(cloud, maxIterations, distanceThreshold);
    //segResult.first = cloudOutliers;
    //segResult.second = cloudInliers;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Separating segmented data took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // create KD-tree object to use as search method
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setInputCloud(cloud);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setClusterTolerance(clusterTolerance);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);

    for (pcl::PointIndices cluster_idx : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr separate_cluster(new pcl::PointCloud<PointT>);

        for (int index : cluster_idx.indices)
        {
            separate_cluster->points.push_back(cloud->points[index]);
        }
        separate_cluster->width = separate_cluster->points.size();
        separate_cluster->height = 1;
        separate_cluster->is_dense = true;

        clusters.push_back(separate_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

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
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr clusterOrig)
{
    // Find the tightest bounding box for the given cluster
    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*clusterOrig, *cluster);

    for (int i = 0; i < cluster->size(); i++)
    {
        cluster->points[i].z = 0;
    }

    Eigen::Vector4f pca_centroid;
    pcl::compute3DCentroid(*cluster, pca_centroid);
    //std::cout << "Before" << pca_centroid << std::endl;
    //pca_centroid[1] = 0;
    pca_centroid[2] = 0;
    //std::cout << "After" << pca_centroid << std::endl;

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cluster, pca_centroid, covariance);
    //std::cout << covariance << std::endl;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    //projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA;
    projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pca_centroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*clusterOrig, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    float length = maxPoint.x - minPoint.x;
    float width = maxPoint.y - minPoint.y;
    float height = maxPoint.z - minPoint.z;

    /*
    Eigen::Matrix3f rotation_matrix(Eigen::Matrix3f::Identity());
    //Eigen::Matrix3f rot_mat(Eigen::Matrix3f::Identity());
    //rot_mat.block<2, 2>(1, 1) = projectionTransform.block<2, 2>(1, 1);
    Eigen::Matrix3f rot_mat_y = eigenVectorsPCA;
    Eigen::Matrix3f rot_mat_z = eigenVectorsPCA;
    Eigen::Matrix3f rot_mat_x = eigenVectorsPCA;
    Eigen::RowVectorXf val_y(3);
    val_y << 0, 1, 0;
    rot_mat_y.block<3, 1>(0, 1) = val_y.transpose();
    rot_mat_y.block<1, 3>(1, 0) = val_y;
    
    Eigen::RowVector3f val_z(3);
    val_z << 1, 0, 0;
    rot_mat_z.block<3, 1>(0, 0) = val_z.transpose();
    rot_mat_z.block<1, 3>(0, 0) = val_z;
    
    Eigen::RowVector3f val_x(3);
    val_x << 0, 0, 1;
    rot_mat_x.block<3, 1>(0, 2) = val_x.transpose();
    rot_mat_x.block<1, 3>(2, 0) = val_x;
    */

    BoxQ Box;
    Box.bboxQuaternion = eigenVectorsPCA;
    Box.bboxTransform = eigenVectorsPCA * meanDiagonal + pca_centroid.head(3);
    Box.cube_height = height;
    Box.cube_length = length;
    Box.cube_width = width;

    return Box;
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
    //std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

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