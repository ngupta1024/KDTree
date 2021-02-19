#include <vector>
#include <memory>

using std::vector;

struct Point2D
{
    int x;
    int y;

	int operator[](int rem) const
	{
		return rem ==0 ? x: y;
	}
	
	//\TODO write operator overload for ostream
	
};

// A templated class of KDTree so that we can accept 2d and 3d points
template<std::size_t K, class T>
class KDTree
{

public:
	struct Node
	{
		Node()
		{
		}

		Node(const T& val, const int& lev):value(val), level(lev)
		{
		}

		std::shared_ptr<Node> left;
		std::shared_ptr<Node> right;
		T value;
		int level = 0;
	};

	std::shared_ptr<Node> root;

    // default constructor
    KDTree();

    // parametrized constructor
    KDTree(const T& rootElem);

    // initialize a KD tree with point cloud
    KDTree(const vector<T>& points);

    // insert a node into KDTree
    // Note: We will overwrite the duplicate nodes 
    void insert(const T& elem);

    // remove a node from KDTree and insert all the elements on that tree again
    // or remove the entire subtree
    //void remove(const T& elem);

    // search for the exact node
    bool search(const T& query, std::shared_ptr<Node> &leafNode) const;

    // nearest neighbor search
    std::shared_ptr<Node> nnSearch(const T& query) const;

private:

std::shared_ptr<Node> buildTree(vector<T> &points, const std::size_t &depth);

void nearestNeighbor(const std::shared_ptr<Node>& root, const T& query, std::size_t depth, std::shared_ptr<Node>& best_node, double &best_distance) const;

};
