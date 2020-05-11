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

	void insertHelper(Node **node, pcl::PointXYZI point, int id, int depth)
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

	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, point, id, 0);
	}

	void searchHelper(Node *node, pcl::PointXYZI target, float distanceTol, int depth, std::vector<int> &ids)
	{
		if (node != NULL)
		{
			// if (fabs(node->point[0] - target[0]) <= distanceTol && fabs(node->point[1] - target[1]) <= distanceTol)

			// if ((node->point[0] <= (target[0]+distanceTol) && node->point[0] >= (target[0]-distanceTol)) && (node->point[1] <= (target[1]+distanceTol) && node->point[1] >= (target[1]-distanceTol)))
			if (fabs(node->point.getArray3fMap()[0] - target.getArray3fMap()[0]) < distanceTol &&
				fabs(node->point.getArray3fMap()[1] - target.getArray3fMap()[1]) < distanceTol &&
				fabs(node->point.getArray3fMap()[2] - target.getArray3fMap()[2]) < distanceTol)
			{
				float distance = sqrt((node->point.getArray3fMap()[0] - target.getArray3fMap()[0]) * (node->point.getArray3fMap()[0] - target.getArray3fMap()[0]) +
									  (node->point.getArray3fMap()[1] - target.getArray3fMap()[1]) * (node->point.getArray3fMap()[1] - target.getArray3fMap()[1]) +
									  (node->point.getArray3fMap()[2] - target.getArray3fMap()[2]) * (node->point.getArray3fMap()[2] - target.getArray3fMap()[2]));
				if (distance < distanceTol)
					ids.push_back(node->id);
			}

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
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, 0, ids);
		return ids;
	}
};
