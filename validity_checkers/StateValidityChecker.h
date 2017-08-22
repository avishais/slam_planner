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

#include "collisions.h"
#include "camera.h"

#define MAP_FILE "./data/Corner_Map_S.txt"
#define UB_FILE "./data/upper_bound.txt"
#define LB_FILE "./data/lower_bound.txt"

#include <iostream>

namespace ob = ompl::base;
using namespace std;

#define PI 3.1416

class StateValidityChecker : public collisionDetection, public camera
{
public:
	StateValidityChecker(const ob::SpaceInformationPtr &si) : mysi_(si.get()), camera(MAP_FILE, UB_FILE, LB_FILE) {
		v.resize(3);
		w.resize(3);
		t.resize(3);
		qt1.resize(3);
		qt2.resize(3);
		qc1.resize(2);
		qc2.resize(2);
		d1mid2.resize(3);
		};
	StateValidityChecker() {
		v.resize(3);
		w.resize(3);
		t.resize(3);
		qt1.resize(3);
		qt2.resize(3);
		qc1.resize(2);
		qc2.resize(2);
		d1mid2.resize(3);
	};

	void retrieveStateVector(const ob::State *state, Vector &a);
	void updateStateVector(const ob::State *state, Vector q);
	void printStateVector(const ob::State *state);

	// Check feasibility of a state
	bool isValid(const ob::State *state);
	bool isValid(Vector q);

	// OMPL check local connection
	bool checkMotion(const ob::State *s1, const ob::State *s2, bool = false);

	// Check local connection of a two-wheels kinematic motion
	bool checkMotionTW(const ob::State *s1, const ob::State *s2);

	// Two-wheels shortest path from two states
	bool GetShortestPath(Vector, Vector);
	bool twb_shortpath(Vector, int, Vector, int, int);
	void twb_gettangent(Vector q1,int s1, Vector q2, int s2);
	Vector myprop(Vector q, double vi, double wi, double ti);
	double twb_getangle(Vector, Vector);

	bool reconstructMotion(const ob::State *, const ob::State *, Matrix &);
	bool reconstructMotionTW(const ob::State *, const ob::State *, Matrix &);

	void defaultSettings();

	/** Misc */
	template <class T>
	void printVector(T);
	void printMatrix(Matrix);
	double normDistance(Vector, Vector, int = -1);

	const int n = 3; // State dimension
	int get_n() {
		return n;
	}


private:
	ob::StateSpace *stateSpace_;
	ob::SpaceInformation    *mysi_;

	/** Robot properties */
	double robot_r = 0.3; // Robot radius
	double turn_radius = 0.25;
	double dt = 0.2; // Interval in which to interpolate the motion

	/** Shortest path parameters */
	VectorInt v;
	Vector w, t;
	Vector qt1, qt2;
	Vector qc1, qc2;
	Vector d1mid2; // {d1cur, dmidcur, d2cur}


};





#endif /* CHECKER_H_ */
