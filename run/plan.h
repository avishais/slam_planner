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
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/config.h>

// Modified and custom planners
#include "../planners/myRRT.h"
#include "../planners/myRRTstar.h"

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

// An enum of available planners
enum plannerType
{
    PLANNER_RRT,
    PLANNER_RRTSTAR
};

// An enum of the supported optimization objectives, alphabetical order
enum planningObjective
{
    OBJECTIVE_PATHLENGTH,
	OBJECTIVE_MINE,
	OBJECTIVE_WEIGHT
};

/** Defines an optimization objective which attempts to maximize the features seen by the camera
 *
    The class StateCostIntegralObjective represents objectives as
    summations of state costs, just like we require. All we need to do
    then is inherit from that base class and define our specific state
    cost function by overriding the stateCost() method.
 */
class myObjective : public ob::StateCostIntegralObjective, StateValidityChecker
{
public:
    myObjective(const ob::SpaceInformationPtr& si) : ob::StateCostIntegralObjective(si, true), StateValidityChecker(si) {}

    ob::Cost stateCost(const ob::State* s) const
    {
    	const ob::RealVectorStateSpace::StateType *Q = s->as<ob::RealVectorStateSpace::StateType>();
    	//double C = Q->values[0]*Q->values[0] + Q->values[1]*Q->values[1];
    	//double C = 1. / fabs(Q->values[1]-(-0.5));
    	int C = countVisible((float)Q->values[0], (float)Q->values[1], (float)Q->values[2]);

    	return ob::Cost(1 / (double)C);
    }
};

// Prototypes
class plan_slam
{
public:
	plan_slam() {};

	bool plan(Vector, Vector, double, plannerType = PLANNER_RRT, planningObjective = OBJECTIVE_PATHLENGTH);

	// Construct the planner specified by our command line argument.
	// This helper function is simply a switch statement.
	ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr, plannerType);

	/** Returns a structure representing the optimization objective to use
	    for optimal motion planning. This method returns an objective
	    which attempts to minimize the length in configuration space of
	    computed paths. */
	ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr&);

	/** Return an optimization objective which attempts to minimiaze turn angle. */
	ob::OptimizationObjectivePtr getMyObjective(const ob::SpaceInformationPtr& si);

	/** Create an optimization objective which attempts to optimize both
	    path length and custom cost function. We do this by defining our individual
	    objectives, then weight them. */
	ob::OptimizationObjectivePtr getWeightedObjective(const ob::SpaceInformationPtr& si);

	 // Create the optimization objective specified by our command-line argument.
	 // This helper function is simply a switch statement.
	ob::OptimizationObjectivePtr allocateObjective(ob::SpaceInformationPtr, planningObjective);

	/** Extract solution path */
	void getPath(ob::ProblemDefinitionPtr pdef, Matrix&);

	bool solved_bool;
	double total_runtime;

	Matrix get_path_matrix() {
		return Path;
	}


private:
	int n = 3;

	Matrix Path;

};

#endif /* plan.h */
