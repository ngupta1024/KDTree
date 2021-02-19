#include "kdtree.hpp"
#include <algorithm>
#include <iostream>
#include <cmath>

using std::cout;
using std::endl;


// default constructor
template<std::size_t K, class T>
KDTree<K,T>::KDTree()
{
}

// parametrized constructor
template<std::size_t K, class T>
KDTree<K,T>::KDTree(const T& rootElem)
{
    root = std::make_shared<Node>(rootElem, 0);
}

// Assumption: all points are unique in the vector
template<std::size_t K, class T>
std::shared_ptr<typename KDTree<K,T>::Node> KDTree<K,T>::buildTree(vector<T> &points, const std::size_t &depth)
{
	if (points.empty()) return nullptr;
	auto comp = [&depth](T& elem1, T& elem2)->bool{
		int axis = depth%K;
		if (elem1[axis] < elem2[axis])
			return true;
		else if (elem1[axis] == elem2[axis])
		{
			return elem1[(depth+1)%K] < elem2[(depth+1)%K];
		}
		return false;
	};
	std::sort(points.begin(), points.end(), comp);	
	auto node = std::make_shared<Node>(points[int(points.size()/2)], depth);
	vector<T> leftVector(points.begin(), points.begin()+int(points.size()/2));
	node->left = buildTree(leftVector, depth+1);
	vector<T> rightVector(points.begin()+int(points.size()/2)+1, points.end());
	node->right = buildTree(rightVector, depth+1);

	return node;
}

template<std::size_t K, class T>
KDTree<K, T>::KDTree(const vector<T>& elems)
{
	auto input = elems;
	root = buildTree(input, 0);	
	if (!root) cout<<"root is still null, there is a bug"<<endl;
}

// insert a node into KDTree
template<std::size_t K, class T>
void KDTree<K,T>::insert(const T& elem)
{
    // check if the tree exists
    if (!root)
    {
        root = std::make_shared<Node>(elem, 0);
		return;
    }

    // check if that elem exists already
    std::shared_ptr<Node> leafNode;
    if (search(elem, leafNode))
        return;

    // if elem does not exist, then add it
    if (!leafNode)
    {
        throw("Some problem with the search function");
    }

    auto newNode = std::make_shared<Node>(elem, leafNode->level + 1);
    if (elem[leafNode->level % K] < leafNode->value[leafNode->level % K])
        leafNode->left = newNode;
    else
        leafNode->right = newNode;
}

// search for the exact node
template<std::size_t K, class T>
bool KDTree<K,T>::search(const T& query, std::shared_ptr<Node> &leafNode) const
{
    auto currNode = root;
    // Iterative approach as recursion can lead to stack overflow
    while(1)
    {
        if (!currNode) return false;
        leafNode = currNode;
        if (query[leafNode->level % K] < leafNode->value[leafNode->level % K])
		{
            currNode = leafNode->left;
		}	
        else if (query[leafNode->level % K] == leafNode->value[leafNode->level % K]) 
		{
			if (query[(leafNode->level+1) % K] < leafNode->value[(leafNode->level+1) % K])
			{
				currNode = leafNode->left;
			}
			else
			{
				if (leafNode->value.x == query.x and leafNode->value.y == query.y) 
					return true;
				currNode = leafNode->right;
			}
		}
		else
		{
            currNode = leafNode->right;
		}
    }
}

template<std::size_t K, class T>
void KDTree<K,T>::nearestNeighbor(const std::shared_ptr<typename KDTree<K,T>::Node>& root, const T& query, std::size_t depth, std::shared_ptr<typename KDTree<K,T>::Node>& best_node, double &best_distance) const
{
	if (!root) return;

	auto distance = std::pow(root->value.x - query.x, 2) + std::pow(root->value.y - query.y, 2); 
    if (distance < best_distance)
    {
        best_distance = distance;
        best_node = root;
    }
    if (best_distance == 0)
        return;

	auto nextBranch = root;
	auto otherBranch = root;
	if (query[depth%K]<root->value[depth%K])
	{
		nextBranch = root->left;
		otherBranch = root->right;
	}
	else if (query[depth%K]==root->value[depth%K])
	{
		if (query[(depth+1)%K]< root->value[(depth+1)%K])
		{
			nextBranch = root->left;
			otherBranch = root->right;
		}
		else
		{
			if (query[(depth+1)%K] == root->value[(depth+1)%K])
			{
				best_node = root;
				best_distance = 0;
				return;	
			}
			nextBranch = root->right;
			otherBranch = root->left;
		}
	}
	else
	{
		nextBranch = root->right;
		otherBranch = root->left;
	}

	cout<<"root = "<<root->value.x<<", "<<root->value.y<<endl;
	if (nextBranch)
	{	
		nearestNeighbor(nextBranch, query, depth+1, best_node, best_distance);

	}

    auto distanceFromRoot = std::pow(root->value.x - query.x, 2) + std::pow(root->value.y - query.y, 2);
    if (distanceFromRoot< distance and otherBranch)
    {
        nearestNeighbor(otherBranch, query, depth+1, best_node, best_distance);
	
        distance = std::pow(nextBranch->value.x - query.x, 2) + std::pow(nextBranch->value.y - query.y, 2); 
        if (distance < best_distance)
        {
            best_distance = distance;
            best_node = nextBranch;
        }
    }
}

// nearest neighbor search
template<std::size_t K, class T>
std::shared_ptr<typename KDTree<K,T>::Node> KDTree<K,T>::nnSearch(const T& query) const
{
	double best_distance = std::numeric_limits<double>::max();
	std::shared_ptr<Node> best_node;
	std::size_t depth =0;
	nearestNeighbor(root, query, depth, best_node, best_distance);
	
	return best_node;
	
}

template class KDTree<2, Point2D>;
