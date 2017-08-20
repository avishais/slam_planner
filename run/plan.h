/*
 * plan.h
 *
 *      Author: Avishai Sintov
 */

#ifndef PLAN_C_SPACE_H_
#define PLAN_C_SPACE_H_

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

// Modified and custom planners
#include "../planners/myRRT.h"

// Standard libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std;

bool isStateValid(const ob::State *state) {
	return true;
}

// Prototypes
class plan_slam
{
public:
	plan_slam() {};

	bool plan(Vector, Vector);

	bool solved_bool;
	double total_runtime;

private:
	int n = 3;

};

#endif /* plan.h */
