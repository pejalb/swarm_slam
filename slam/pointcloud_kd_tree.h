#ifndef KD_TREE_SEARCH_
#define KD_TREE_SEARCH_
#include "nanoflann.h"

#include <ctime>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include "ponto.h"

using namespace std;
using namespace nanoflann;

// This is an example of a custom data set class
typedef ponto Point;



template <typename T>
struct PointCloud
{
    //struct Point
    //{
    //T  x,y,z;
    //};

    std::vector<Point>  pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline T kdtree_distance(const T *p1, const size_t idx_p2, size_t /*size*/) const
    {
        const T d0 = p1[0] - pts[idx_p2].x;
        const T d1 = p1[1] - pts[idx_p2].y;
        return d0*d0 + d1*d1;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim == 0)
            return pts[idx].x;
        else
            return pts[idx].y;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

};

typedef KDTreeSingleIndexAdaptor<
    L2_Simple_Adaptor<double, PointCloud<double> >,
    PointCloud<double>,
    2 /* dim */
> my_kd_tree_t;

template <typename T>
void generateRandomPointCloud(PointCloud<T> &point, const size_t N, const T max_range = 10)
{
    std::cout << "Generating " << N << " point cloud...";
    point.pts.resize(N);
    for (size_t i = 0; i<N; i++)
    {
        point.pts[i].x = max_range * (rand() % 1000) / T(1000);
        point.pts[i].y = max_range * (rand() % 1000) / T(1000);
        //point.pts[i].angulo = max_range * (rand() % 1000) / T(1000);
    }

    std::cout << "done\n";
}

template <typename num_t>
void kdtree_demo(const size_t N)
{
    PointCloud<num_t> cloud;

    // Generate points:
    generateRandomPointCloud(cloud, N);

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<num_t, PointCloud<num_t> >,
        PointCloud<num_t>,
        2 /* dim */
    > my_kd_tree_t;

    my_kd_tree_t   index(2 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    index.buildIndex();

#if 0
    // Test resize of dataset and rebuild of index:
    cloud.pts.resize(cloud.pts.size()*0.5);
    index.buildIndex();
#endif

    const num_t query_pt[3] = { 0.5, 0.5 };

    // ----------------------------------------------------------------
    // knnSearch():  Perform a search for the N closest points
    // ----------------------------------------------------------------
    {
        size_t num_results = 5;
        std::vector<size_t>   ret_index(num_results);
        std::vector<num_t> out_dist_sqr(num_results);

        num_results = index.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

        // In case of less points in the tree than requested:
        ret_index.resize(num_results);
        out_dist_sqr.resize(num_results);

        cout << "knnSearch(): num_results=" << num_results << "\n";
        for (size_t i = 0; i<num_results; i++)
            cout << "idx[" << i << "]=" << ret_index[i] << " dist[" << i << "]=" << out_dist_sqr[i] << endl;
        cout << "\n";
    }

    // ----------------------------------------------------------------
    // radiusSearch():  Perform a search for the N closest points
    // ----------------------------------------------------------------
    {
        const num_t search_radius = static_cast<num_t>(0.1);
        std::vector<std::pair<size_t, num_t> >   ret_matches;

        nanoflann::SearchParams params;
        //params.sorted = false;

        const size_t nMatches = index.radiusSearch(&query_pt[0], search_radius, ret_matches, params);

        cout << "radiusSearch(): radius=" << search_radius << " -> " << nMatches << " matches\n";
        for (size_t i = 0; i<nMatches; i++)
            cout << "idx[" << i << "]=" << ret_matches[i].first << " dist[" << i << "]=" << ret_matches[i].second << endl;
        cout << "\n";
    }

}

template <typename num_t>
std::vector<std::vector<size_t> > constroiKDtree(PointCloud<num_t> &cloud, std::vector<ponto> &pontos, size_t num_results = 5)
{
    // construct a kd-tree index:
    // typedef KDTreeSingleIndexAdaptor<
    //     L2_Simple_Adaptor<num_t, PointCloud<num_t> >,
    //     PointCloud<num_t>,
    //     2 /* dim */
    // > my_kd_tree_t;

    my_kd_tree_t   index(2 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    index.buildIndex();

    //const num_t query_pt[2] = { 1.0, 0.5 };

    // ----------------------------------------------------------------
    // knnSearch():  Perform a search for the N closest points
    // ----------------------------------------------------------------
    {
        //size_t num_results = 5;
        std::vector<std::vector<size_t> >   ret_indexes; ret_indexes.reserve(pontos.size());
        //garante espaco
        std::vector<size_t>   ret_index(num_results);
        std::vector<num_t> out_dist_sqr(num_results);
        num_t query_pt[2];
        for (int i = 0; i < pontos.size(); i++){
            //ret_indexes[i].reserve(num_results);
            query_pt[0] = pontos[i].x;
            query_pt[1] = pontos[i].y;
            num_results = index.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);
            ret_indexes.push_back(ret_index);
        }
        //num_results = index.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);
        // In case of less points in the tree than requested:
        ret_index.resize(num_results);
        //out_dist_sqr.resize(num_results);
        return ret_indexes;
    }
}

template <typename num_t>
std::vector<size_t> constroiKDtree(PointCloud<num_t> &cloud, ponto p, size_t num_results = 5)
{
	// construct a kd-tree index:
	// typedef KDTreeSingleIndexAdaptor<
	//     L2_Simple_Adaptor<num_t, PointCloud<num_t> >,
	//     PointCloud<num_t>,
	//     2 /* dim */
	// > my_kd_tree_t;

	my_kd_tree_t   index(2 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
	index.buildIndex();

	// ----------------------------------------------------------------
	// knnSearch():  Perform a search for the N closest points
	// ----------------------------------------------------------------
		//size_t num_results = 5;
		//garante espaco
	std::vector<size_t>   ret_index(num_results);
	std::vector<num_t> out_dist_sqr(num_results);
	num_t query_pt[2];
	query_pt[0] = p.x;
	query_pt[1] = p.y;
	num_results = index.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

	//num_results = index.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);
	// In case of less points in the tree than requested:
	ret_index.resize(num_results);
	//out_dist_sqr.resize(num_results);
	return ret_index;
}
#endif

/* int main()
{

// Randomize Seed
srand(time(NULL));
PointCloud<double> cloud;
// Generate points:
generateRandomPointCloud(cloud, 541);
double query_pt[2] = { 1.0,1.0 };
std::vector<size_t> knn = constroiKDtree<double>(cloud, query_pt);
//kdtree_demo<double>(4);
//kdtree_demo<double>(100000);
return 0;
}*/


