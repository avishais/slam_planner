/*
 * Checker.h
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

#ifndef CHECKER_H_
#define CHECKER_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include "ompl/base/MotionValidator.h"
#include "ompl/base/State.h"
#include <ompl/config.h>

#include "camera.h"
#include "collisions.h"

#include <iostream>

namespace ob = ompl::base;
using namespace std;

#define PI 3.1416

#define MAP_FILE "./data/Corner_Map_S.txt"
#define UB_FILE "./data/upper_bound.txt"
#define LB_FILE "./data/lower_bound.txt"

typedef struct {
	bool valid;
	VectorInt v;
	Vector w;
	Vector t;
} TW_path_data;

typedef struct {
	bool valid;
	Vector qt1;
	Vector qt2;
	Vector d1mid2;
} TW_tangent;

class StateValidChecker : public collisionDetection, public camera
{
public:
	StateValidChecker(const ob::SpaceInformationPtr &si) : mysi_(si.get()), camera(MAP_FILE, UB_FILE, LB_FILE) {};
	StateValidChecker() {};

	void retrieveStateVector(const ob::State *state, Vector &a);
	void updateStateVector(const ob::State *state, Vector q);
	void printStateVector(const ob::State *state);

	// Check feasibility of a state
	bool isValid(const ob::State *state);
	bool isValid(Vector q);

	// OMPL check local connection
	bool checkMotion(const ob::State *s1, const ob::State *s2);

	// Check local connection of a two-wheels kinematic motion
	bool checkMotionTW(const ob::State *s1, const ob::State *s2);

	// Two-wheels shortest path from two states
	TW_path_data GetShortestPath(Vector, Vector) const;
	TW_tangent twb_shortpath(Vector, int, Vector, int, int) const;
	TW_tangent twb_gettangent(Vector q1,int s1, Vector q2, int s2) const;
	Vector myprop(Vector q, double vi, double wi, double ti) const;
	double twb_getangle(Vector, Vector) const;

	bool reconstructMotion(const ob::State *, const ob::State *, Matrix &);
	bool reconstructMotionTW(const ob::State *, const ob::State *, Matrix &);

	void defaultSettings();

	/** Misc */
	template <class T>
	void printVector(T);
	void printMatrix(Matrix);
	double normDistance(Vector, Vector, int = -1) const;

	const int n = 3; // State dimension
	int get_n() {
		return n;
	}

	double MotionCost(Matrix) const;
	double MotionCost(const ob::State *, const ob::State *) const;

private:
	ob::StateSpace *stateSpace_;
	ob::SpaceInformation    *mysi_;

	/** Robot properties */
	double robot_r = 0.3; // Robot radius
	double turn_radius = 0.25;
	double dt = 0.2; // Interval in which to interpolate the motion
};





#endif /* CHECKER_H_ */
