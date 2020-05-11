/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "./render/render.h"

// Structure to represent node of kd tree
//template<typename PointT>  //-----------------------_>>>>>>>>>>>>>>>>>>CHANGE DEF TO TEMPLATE DEFINITION
//typedef pcl::PointXYZI PointT;
struct Node
{
	//std::vector<float> point;
	pcl::PointXYZI point;
	int id;
	Node *left;
	Node *right;

	Node(pcl::PointXYZI arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

//template<typename PointT>  //-----------------------_>>>>>>>>>>>>>>>>>>CHANGE DEF TO TEMPLATE DEFINITION
struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	// Insert points in tree
	template <typename PointT>
	void insertHelper(Node **node, PointT point, int id, int depth)
	{
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			int check_index = depth % 3;

			if (point.getArray3fMap()[check_index] >= (*node)->point.getArray3fMap()[check_index])
			{
				insertHelper(&((*node)->right), point, id, depth + 1);
			}
			else
			{
				insertHelper(&((*node)->left), point, id, depth + 1);
			}
		}
	}

	template <typename PointT>
	void insert(PointT point, int id)
	{
		insertHelper(&root, point, id, 0);
	}

	template <typename PointT>
	void searchHelper(Node *node, PointT target, float distanceTol, int depth, std::vector<int> &ids)
	{
		if (node != NULL)
		{	
			// Box check - initial distance check
			if (fabs(node->point.getArray3fMap()[0] - target.getArray3fMap()[0]) < distanceTol &&
				fabs(node->point.getArray3fMap()[1] - target.getArray3fMap()[1]) < distanceTol &&
				fabs(node->point.getArray3fMap()[2] - target.getArray3fMap()[2]) < distanceTol)
			{
				// Compute distance when inside box
				float distance = sqrt((node->point.getArray3fMap()[0] - target.getArray3fMap()[0]) * (node->point.getArray3fMap()[0] - target.getArray3fMap()[0]) +
									  (node->point.getArray3fMap()[1] - target.getArray3fMap()[1]) * (node->point.getArray3fMap()[1] - target.getArray3fMap()[1]) +
									  (node->point.getArray3fMap()[2] - target.getArray3fMap()[2]) * (node->point.getArray3fMap()[2] - target.getArray3fMap()[2]));
				
				// Record index with qualified distances
				if (distance < distanceTol)
					ids.push_back(node->id);
			}

			// Route the progression inside the tree with the help of depth
			int check_index = depth % 3;
			if ((target.getArray3fMap()[check_index] - distanceTol) < node->point.getArray3fMap()[check_index])
			{
				searchHelper(node->left, target, distanceTol, depth + 1, ids);
			}
			if ((target.getArray3fMap()[check_index] + distanceTol) > node->point.getArray3fMap()[check_index])
			{
				searchHelper(node->right, target, distanceTol, depth + 1, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	template <typename PointT>
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, 0, ids);
		return ids;
	}
};
