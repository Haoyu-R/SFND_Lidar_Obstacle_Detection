/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node<PointT>**node, int depth, PointT point, int id) {

		if (*node == NULL) {
			*node = new Node<PointT>(point, id);
		}
		else {
			int cd = depth % 2;

			if (cd == 0) {
				if (point.x < (*node)->point.x) {
					insertHelper(&((*node)->left), depth++, point, id);
				}
				else {
					insertHelper(&((*node)->right), depth++, point, id);
				}
			}
			else if (cd == 1) {
				if (point.y < (*node)->point.y) {
					insertHelper(&((*node)->left), depth++, point, id);
				}
				else {
					insertHelper(&((*node)->right), depth++, point, id);
				}
			}
			else {
				if (point.z < (*node)->point.z) {
					insertHelper(&((*node)->left), depth++, point, id);
				}
				else {
					insertHelper(&((*node)->right), depth++, point, id);
				}
			}
		}
			
	}

	void insert(PointT point, int id)
	{

		insertHelper(&root, 0, point, id);
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

	}

	// return a list of point ids in the tree that are within distance of target

	void searchHelper(Node<PointT> **node, int depth, const PointT& target, const PointT &box_min, const PointT &box_max, std::vector<int> &ids, const float &distanceTol) {
		
		if (*node == NULL)
			return;

		PointT point = (*node)->point;

		int cd = depth % 2;

		if (point.x > box_max.x || point.x < box_min.x || point.y > box_max.y || point.y < box_min.y || point.z > box_max.z || point.z < box_min.z){
			
		}
		else {
			float dist = 0;

			dist += std::pow(point.x - target.x, 2);
			dist += std::pow(point.y - target.y, 2);
			dist += std::pow(point.z - target.z, 2);

			if (sqrt(dist) <= distanceTol)
				ids.push_back((*node)->id);
		}

		if (cd == 0) {
			if (target.x - distanceTol < point.x) {
				searchHelper(&((*node)->left), depth++, target, box_min, box_max, ids, distanceTol);
			}

			if (target.x + distanceTol > point.x) {
				searchHelper(&((*node)->right), depth++, target, box_min, box_max, ids, distanceTol);
			}
		}
		else if (cd == 1) {
			if (target.y - distanceTol < point.y) {
				searchHelper(&((*node)->left), depth++, target, box_min, box_max, ids, distanceTol);
			}

			if (target.y + distanceTol > point.y) {
				searchHelper(&((*node)->right), depth++, target, box_min, box_max, ids, distanceTol);
			}
		}
		else {
			if (target.z - distanceTol < point.z) {
				searchHelper(&((*node)->left), depth++, target, box_min, box_max, ids, distanceTol);
			}

			if (target.z + distanceTol > point.z) {
				searchHelper(&((*node)->right), depth++, target, box_min, box_max, ids, distanceTol);
			}
		}


	}


	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;

		PointT box_min = target;
		PointT box_max = target;


		box_min.x -= distanceTol;
		box_min.y -= distanceTol;
		box_min.z -= distanceTol;
		box_max.x += distanceTol;
		box_max.y += distanceTol;
		box_max.z += distanceTol;
	

		searchHelper(&root, 0, target, box_min, box_max, ids, distanceTol);

		return ids;
	}
	

};