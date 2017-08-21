/*
 * collisions.h
 *
 *  Created on: Aug 18, 2017
 *      Author: avishai
 */

#ifndef VALIDITYCHECK_COLLISIONS_H_
#define VALIDITYCHECK_COLLISIONS_H_


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// For nn-search
#include "./nanoflann.hpp"
#include "./KDTreeVectorOfVectorsAdaptor.h"

typedef std::vector<std::vector< double > > Matrix;
typedef std::vector< double > Vector;
typedef std::vector< int > VectorInt;
typedef KDTreeVectorOfVectorsAdaptor< Matrix, double > my_kd_tree_t;

using namespace std;
using namespace nanoflann;

typedef struct {
	double x;
	double y;

	double r; // Radius of obstacle
} obst;

struct kNeighborSoln {
	VectorInt neighbors;
	Vector dist;
};

class collisionDetection
{
public:
	collisionDetection();

	void load_obstacles();

	bool check_collisions(Vector, double);

	void print_obs(obst o);

	void kNeighbors(my_kd_tree_t&, Vector, kNeighborSoln&, size_t, bool = false);


private:
	vector <obst> obs;
	Matrix Obs;
	double obs_r = 0.05; // Assumed point obstacle radius

	string path_file = "./data/obs.txt";

	int num_of_obs;

	my_kd_tree_t KDtree;
};

#endif /* VALIDITYCHECK_COLLISIONS_H_ */
