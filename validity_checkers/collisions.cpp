/*
 * collisions.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: avishai
 */

#include "collisions.h"

collisionDetection::collisionDetection() : KDtree(2, {{0,0},{1,1}}, 1) { // Dummy construction of the kd-tree class

	load_obstacles();

	// Re-construction of the kd-tree with the real data set
	KDtree.~KDTreeVectorOfVectorsAdaptor();
	new(&KDtree) my_kd_tree_t(2, Obs, 10);
	//KDtree = my_kd_tree_t(2, Obs, 10); // This is another option that did not work

	KDtree.index->buildIndex();

}

void collisionDetection::load_obstacles() {

	obst o_temp;

	ifstream fq;
	fq.open(path_file);

	int i = 0;
	while(!fq.eof()) {
		fq >> o_temp.x;
		fq >> o_temp.y;
		fq >> o_temp.r;
		obs.push_back(o_temp);
		Obs.push_back({o_temp.x, o_temp.y});
		i++;
	}
	fq.close();

	num_of_obs = obs.size();

}

void collisionDetection::print_obs(obst o) {
	cout << "x: " << o.x << ", y: " << o.y << ", radius: " <<  o.r << endl;
}

bool collisionDetection::check_collisions(Vector q, double robot_radius) {
	// Returns true if robot is not in collision with the obstacles

	if (num_of_obs == 0)
		return true;

	// Brute-force collision checking
	/*for (int i = 0; i < num_of_obs; i++) {
		if ( ((q[0]-obs[i].x)*(q[0]-obs[i].x) + (q[1]-obs[i].y)*(q[1]-obs[i].y)) <= (robot_radius + obs[i].r)*(robot_radius + obs[i].r) )
			return false;
	}*/

	// Collision checking using kd-tree
	kNeighborSoln soln;
	kNeighbors(KDtree, {q[0], q[1]}, soln, 1);

	if (soln.dist[0] <= robot_radius + obs_r)
		return false;

	return true;
}

void collisionDetection::kNeighbors(my_kd_tree_t& mat_index, Vector query, kNeighborSoln& soln, size_t num_results, bool remove_1st_neighbor){
	// find nearest neighbors for node i in configs, which are 6-D A's.

	// do a knn search
	if (remove_1st_neighbor)
		num_results += 1;

	vector<size_t> ret_indexes(num_results);
	Vector out_dists_sqr(num_results);

	nanoflann::KNNResultSet<double> resultSet(num_results);
	resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

	Vector query_pnt = query;
	mat_index.index->findNeighbors(resultSet, &query_pnt[0], SearchParams(10));

	VectorInt rtn_values(ret_indexes.begin(), ret_indexes.end());

	if (remove_1st_neighbor) {
		rtn_values.erase(rtn_values.begin()); // Remove first node that is itself.
		out_dists_sqr.erase(out_dists_sqr.begin()); // Remove first node that is itself.
	}

	Vector out_dists(out_dists_sqr.size());
	for (int i = 0; i < out_dists_sqr.size(); i++)
		out_dists[i] = sqrt(out_dists_sqr[i]);

	// If some error pops, try this
	//soln.neighbors.resize(rtn_values.size());
	//soln.dist.resize(rtn_values.size());

	soln.neighbors = rtn_values;
	soln.dist = out_dists;
}


/*
int main() {
	collisionDetection cd;

	Vector q = {-1.0059, 0.957, 0};

	cout << cd.check_collisions(q, 0.00001) << endl;

	return 0;
}*/


