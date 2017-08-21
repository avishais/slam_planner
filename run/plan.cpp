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

bool plan_slam::plan(Vector q_start, Vector q_goal) {

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

	 // create a planner for the defined space
	 // To add a planner, the #include library must be added above
	 ob::PlannerPtr planner(new og::RRT(si));

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
	 pdef->print(std::cout); // Prints problem definition such as start and goal states and optimization objective

	 // attempt to solve the problem within one second of planning time
	 clock_t st = clock();
	 ob::PlannerStatus solved = planner->solve(1.0);
	 double runtime = double(clock() - st) / CLOCKS_PER_SEC;

	if (solved) {
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		ob::PathPtr path = pdef->getSolutionPath();
		std::cout << "Found solution in " << runtime << " seconds." << std::endl;

		// print the path to screen
		path->print(std::cout);  // Print as vectors

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

int main(int, char **) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

	Vector q_start = {0, 0, 0};
	Vector q_goal = {5.584, -2.0431, -1.5707};

	plan_slam pl;

	clock_t st = clock();
	pl.plan(q_start, q_goal);
	double runtime = double(clock() - st) / CLOCKS_PER_SEC;
	std::cout << "Net runtime: " << runtime << " seconds." << std::endl;

	std::cout << std::endl << std::endl;

	return 0;
}
