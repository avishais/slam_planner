/*
 * myOptimization.h
 *
 *  Created on: Aug 22, 2017
 *      Author: avishai
 */

#ifndef VALIDITY_CHECKERS_MYOPTIMIZATION_H_
#define VALIDITY_CHECKERS_MYOPTIMIZATION_H_

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/config.h>

// Modified and custom planners
#include "../validity_checkers/StateValidChecker.h"

// Standard libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>

namespace ob = ompl::base;
using namespace ob;

/** Defines an optimization objective which attempts to minimize the length of the path in the R^2 X S configuration space
 *
    The class StateCostIntegralObjective represents objectives as
    summations of state costs, just like we require. All we need to do
    then is inherit from that base class and define our specific state
    cost function by overriding the stateCost() method.
 */
class LengthObjective : public ob::StateCostIntegralObjective, public StateValidChecker
{
public:
	LengthObjective(const ob::SpaceInformationPtr& si) : ob::StateCostIntegralObjective(si, true), StateValidChecker(si) {cout << "Init custom objective function.\n";}

    ob::Cost stateCost(const ob::State* s) const
    {
      	return identityCost();
    }

    ob::Cost motionCost(const State *s1, const State *s2) const
    {
    	return ob::Cost(MotionCost(s1, s2));
    }

    ob::Cost motionCostHeuristic(const State *s1, const State *s2) const
    {
    	return motionCost(s1, s2);
    }

    int thereshold = 5;
};


/** Defines an optimization objective which attempts to maximize the features seen by the camera
 *
    The class StateCostIntegralObjective represents objectives as
    summations of state costs, just like we require. All we need to do
    then is inherit from that base class and define our specific state
    cost function by overriding the stateCost() method.
 */
class CameraObjective : public ob::StateCostIntegralObjective, public StateValidChecker
{
public:
	CameraObjective(const ob::SpaceInformationPtr& si) : ob::StateCostIntegralObjective(si, true), StateValidChecker(si) {cout << "Init custom objective function.\n";}

    ob::Cost stateCost(const ob::State* s) const
    {

    	const ob::RealVectorStateSpace::StateType *Q = s->as<ob::RealVectorStateSpace::StateType>();
    	int C = countVisible((float)Q->values[0], (float)Q->values[1], (float)Q->values[2]);

    	double cost = C < thereshold ? 1e-5 : (double)C;

    	//cout << (float)Q->values[0] << " " << (float)Q->values[1] << " " << (float)Q->values[2] << endl;

    	return ob::Cost(1 / cost);
    }

    /*ob::Cost motionCost(const State *s1, const State *s2) const
    {
    	return ob::Cost(MotionCost(s1, s2));
    }

    ob::Cost motionCostHeuristic(const State *s1, const State *s2) const
    {
    	return motionCost(s1, s2);
    }*/

    int thereshold = 5;
};


#endif /* VALIDITY_CHECKERS_MYOPTIMIZATION_H_ */
