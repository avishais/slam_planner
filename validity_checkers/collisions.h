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

typedef std::vector<std::vector< double > > Matrix;
typedef std::vector< double > Vector;
typedef std::vector< int > VectorInt;

using namespace std;

typedef struct {
	double x;
	double y;

	double r; // Radius of obstacle
} obst;

class collisionDetection
{
public:
	collisionDetection();

	void load_obstacles();

	bool check_collisions(Vector, double);

	void print_obs(obst o);

private:
	vector <obst> obs;

	string path_file = "./data/obs.txt";

	int num_of_obs;
};

#endif /* VALIDITYCHECK_COLLISIONS_H_ */
