#include <iostream>
#include "kdtree.hpp"

int main()
{
	using std::cout;
	using std::endl;
	const std::size_t K =2;

//Test case 1
	auto kd0 = std::make_unique<KDTree<K,Point2D>>(Point2D{2,3});
	if (!kd0->root)
		cout<<"nullptr"<<endl;
	else
		cout<<kd0->root->level<<endl;
	kd0->insert(Point2D{1,2});	
	kd0->insert(Point2D{1,2});	

//Test case 2 nullptr
	auto kd1 = std::make_unique<KDTree<K,Point2D>>();
	if (!kd1->root)
		cout<<"nullptr"<<endl;
	else
		cout<<kd1->root->level<<endl;
	kd1->insert(Point2D{1,2});	
	kd1->insert(Point2D{1,2});	
	
//Test case 3
	std::vector<Point2D> points;
	for (int i=0; i<3; ++i)
	{
		for (int j=0; j<3; ++j)
		{
			points.push_back(Point2D{i,j});
		}
	}
	auto kd2 = std::make_unique<KDTree<K,Point2D>>(points);
	std::shared_ptr<KDTree<K, Point2D>::Node> leafNode;
    std::cout<<"Expected: 0, Actual: "<<kd2->search(Point2D{0,3}, leafNode)<<std::endl;	
    std::cout<<"Expected: 1, Actual: "<<kd2->search(Point2D{2,2}, leafNode)<<std::endl;	

//Test case 4

	std::vector<Point2D> randPts{Point2D{0,0}, Point2D{0,2}, Point2D{2,0}};
	auto kd3 = std::make_unique<KDTree<K,Point2D>>(randPts);
	auto res = kd3->nnSearch(Point2D{0,0});
	cout<<res->value.x<<" ,"<< res->value.y<<endl;
	return 0;
}
