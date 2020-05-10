/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	void insertHelper(Node **node, std::vector<float> point, int id, int depth)
	{
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			int check_index = depth % 2;

			if (point[check_index] > (*node)->point[check_index])
			{
				insertHelper(&((*node)->right), point, id, depth + 1);
			}
			else
			{
				insertHelper(&((*node)->left), point, id, depth + 1);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, point, id, 0);
	}

	void searchHelper(Node* node, std::vector<float> target, float distanceTol, int depth, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			// if (fabs(node->point[0] - target[0]) <= distanceTol && fabs(node->point[1] - target[1]) <= distanceTol)
			
			// if ((node->point[0] <= (target[0]+distanceTol) && node->point[0] >= (target[0]-distanceTol)) && (node->point[1] <= (target[1]+distanceTol) && node->point[1] >= (target[1]-distanceTol)))
			if (fabs(node->point[0] - target[0]) <= distanceTol && fabs(node->point[1] - target[1]) <= distanceTol)
			{
				float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]));
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}
			
			int check_index = depth % 2;
			if ((target[check_index] - distanceTol) < node->point[check_index])
			{
				searchHelper(node->left, target, distanceTol, depth+1, ids);
			}
			if ((target[check_index] + distanceTol) > node->point[check_index])
			{
				searchHelper(node->right, target, distanceTol, depth+1, ids);
			}
		}
		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, 0, ids);
		return ids;
	}
};
