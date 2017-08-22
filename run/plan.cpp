/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Avishai Sintov */

#include "plan.h"

ob::PlannerPtr plan_slam::allocatePlanner(ob::SpaceInformationPtr si, plannerType p_type)
{
    switch (p_type)
    {
        case PLANNER_RRT:
        {
            return std::make_shared<og::RRT>(si);
            break;
        }
        case PLANNER_RRTSTAR:
        {
            return std::make_shared<og::RRTstar>(si);
            break;
        }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}

/** Returns a structure representing the optimization objective to use
    for optimal motion planning. This method returns an objective
    which attempts to minimize the length in configuration space of
    computed paths. */
ob::OptimizationObjectivePtr plan_slam::getPathLengthObjective(const ob::SpaceInformationPtr& si)
{

    OMPL_INFORM("Loading path length optimization objective.");

    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

/** Return an optimization objective which attempts to minimiaze turn angle. */
ob::OptimizationObjectivePtr plan_slam::getMyObjective(const ob::SpaceInformationPtr& si)
{
    OMPL_INFORM("Loading custom optimization objective.");

    return ob::OptimizationObjectivePtr(new myObjective(si));
}

/** Create an optimization objective equivalent to the one returned by
    getBalancedObjective1(), but use an alternate syntax. */
ob::OptimizationObjectivePtr plan_slam::getWeightedObjective(const ob::SpaceInformationPtr& si)
{
    OMPL_INFORM("Loading multi-objective optimization.");

    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr customObj(new myObjective(si));

    return 1.0*lengthObj + customObj;
}

ob::OptimizationObjectivePtr plan_slam::allocateObjective(ob::SpaceInformationPtr si, planningObjective objectiveType)
{
    switch (objectiveType)
    {
        case OBJECTIVE_PATHLENGTH:
            return getPathLengthObjective(si);
            break;
        case OBJECTIVE_MINE:
            return getMyObjective(si);
            break;
        case OBJECTIVE_WEIGHT:
        	return getWeightedObjective(si);
        	break;
        default:
            OMPL_ERROR("Optimization-objective enum is not implemented in allocation function.");
            return ob::OptimizationObjectivePtr();
            break;
    }
}

void plan_slam::getPath(ob::ProblemDefinitionPtr pdef, Matrix &M) {

	og::PathGeometric Path( dynamic_cast< const og::PathGeometric& >( *pdef->getSolutionPath()));
	const std::vector< ob::State* > &states = Path.getStates();
	ob::State *state;
	Vector q(3);
	for( size_t i = 0 ; i < states.size( ) ; ++i ) {
		state = states[i]->as< ob::State >();
		q[0] = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
		q[1] = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
		q[2] = state->as<ob::RealVectorStateSpace::StateType>()->values[2];
		M.push_back(q);
	}
}

bool plan_slam::plan(Vector q_start, Vector q_goal, double runtime, plannerType p_type, planningObjective o_type) {

	// construct the state space we are planning in
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(n)); // A-space - state space of the rod - R^6

	// set the bounds for the A=R^6
	ob::RealVectorBounds Qbounds(n);
	Qbounds.setLow(0, -1.15); // x_min
	Qbounds.setHigh(0, 6.5); // x_max
	Qbounds.setLow(1, -5); // y_min
	Qbounds.setHigh(1, 2); // y_max
	Qbounds.setLow(2, -3.14); // theta_min
	Qbounds.setHigh(2, 3.14); // theta_max

	// set the bound for the space
	Q->as<ob::RealVectorStateSpace>()->setBounds(Qbounds);

	// construct a compound state space using the overloaded operator+
	ob::StateSpacePtr Qspace(Q);

	 // construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(Qspace));

	 // set state validity checking for this space
	//si->setStateValidityChecker(ob::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
	si->setStateValidityCheckingResolution(0.03); // 3% ???

	// create a random start state
	ob::ScopedState<ob::RealVectorStateSpace> start(Qspace);
	for (int i = 0; i < n; i++)
		start->as<ob::RealVectorStateSpace::StateType>()->values[i] = q_start[i];

	 // create a random goal state
	ob::ScopedState<ob::RealVectorStateSpace> goal(Qspace);
	for (int i = 0; i < n; i++)
		goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = q_goal[i];

	 // create a problem instance
	 ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	 // set the start and goal states
	 pdef->setStartAndGoalStates(start, goal);

	 // If this is an optimizing planner, set the optimization objective
	 if (p_type==PLANNER_RRTSTAR) {
		 // Create the optimization objective specified by our command-line argument.
		 // This helper function is simply a switch statement.
		 pdef->setOptimizationObjective(allocateObjective(si, o_type));
	 }

	 // create a planner for the defined space
	 // To add a planner, the #include library must be added above
	 ob::PlannerPtr planner = allocatePlanner(si, p_type);

	 // set the problem we are trying to solve for the planner
	 planner->setProblemDefinition(pdef);

	 // perform setup steps for the planner
	 planner->setup();

	 //planner->printSettings(std::cout); // Prints some parameters such as range
	 //planner->printProperties(std::cout); // Prints some decisions such as multithreading, display approx solutions, and optimize?

	 // print the settings for this space
	 si->printSettings(std::cout); // Prints state space settings such as check resolution, segmant count factor and bounds
	 //si->printProperties(std::cout); // Prints state space properties, average length, dimension ...

	 // print the problem settings
	 //pdef->print(std::cout); // Prints problem definition such as start and goal states and optimization objective

	 // attempt to solve the problem within one second of planning time
	 clock_t st = clock();
	 ob::PlannerStatus solved = planner->solve(runtime);
	 double Ttime = double(clock() - st) / CLOCKS_PER_SEC;

	if (solved) {
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		//ob::PathPtr path = pdef->getSolutionPath();
		std::cout << "Found solution in " << Ttime << " seconds." << std::endl;

		getPath(pdef, Path);
		//StateValidityChecker svc;
		//svc.printMatrix(Path);

		// print the path to screen
		//path->print(std::cout);  // Print as vectors

		// Save path to file
		//std::ofstream myfile;
		//myfile.open("./paths/path.txt");
		//og::PathGeometric& pog = static_cast<og::PathGeometric&>(*path); // Transform into geometric path class
		//pog.printAsMatrix(myfile); // Print as matrix to file
		//myfile.close();
	}
	 else
	 std::cout << "No solution found" << std::endl;
}

// Run planner in this form: ./<exe-file> <runtime> <planner type> <optimizer type>
int main(int argn, char ** args) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	double runtime;
	plannerType p_type;
	planningObjective o_type;

	if (argn == 1) {
		runtime = 1; // sec
		p_type = PLANNER_RRT;
	}
	else if (argn == 2) {
		runtime = atof(args[1]);
		p_type = PLANNER_RRT;
	}
	else {
		runtime = atof(args[1]);
		p_type = atoi(args[2])==1 ? PLANNER_RRT : PLANNER_RRTSTAR;
		if (argn==4)
			o_type = atoi(args[3])==1 ? OBJECTIVE_PATHLENGTH : OBJECTIVE_MINE;
		else
			o_type = OBJECTIVE_PATHLENGTH;
	}

	Vector q_start = {0, 0, 3.14/4};
	Vector q_goal = {5.584, -2.0431, -1.5707};

	plan_slam pl;

	clock_t st = clock();
	pl.plan(q_start, q_goal, runtime, p_type, o_type);
	double Ntime = double(clock() - st) / CLOCKS_PER_SEC;
	std::cout << "Net runtime: " << Ntime << " seconds." << std::endl;

	std::cout << std::endl << std::endl;

	return 0;
}
