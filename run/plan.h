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
    OBJECTIVE_PATHTURN
};

/** Defines an optimization objective which attempts to minimize the turn angle
 *
    The class StateCostIntegralObjective represents objectives as
    summations of state costs, just like we require. All we need to do
    then is inherit from that base class and define our specific state
    cost function by overriding the stateCost() method.
 */
class turnObjective : public ob::StateCostIntegralObjective
{
public:
    turnObjective(const ob::SpaceInformationPtr& si) : ob::StateCostIntegralObjective(si, true) {}

    ob::Cost stateCost(const ob::State* s) const
    {
    	const ob::RealVectorStateSpace::StateType* state =
    			state->as<ob::RealVectorStateSpace::StateType>();

    	// Extract the robot's (x,y) position from its state
    	double theta = state->values[2];
    	return ob::Cost(1 / theta);
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
	ob::OptimizationObjectivePtr getTurnObjective(const ob::SpaceInformationPtr& si);

	 // Create the optimization objective specified by our command-line argument.
	 // This helper function is simply a switch statement.
	ob::OptimizationObjectivePtr allocateObjective(ob::SpaceInformationPtr, planningObjective);


	bool solved_bool;
	double total_runtime;

private:
	int n = 3;

};

#endif /* plan.h */
