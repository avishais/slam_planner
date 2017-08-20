/*
 * collisions.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: avishai
 */

#include "collisions.h"

collisionDetection::collisionDetection() {

	load_obstacles();

}

void collisionDetection::load_obstacles() {

	obst o_temp;

	ifstream fq;
	fq.open(path_file);

	int i = 0;
	while(!fq.eof()) {
		fq >> o_temp.x;
		fq >> o_temp.y;
		fq >> o_temp.r;
		obs.push_back(o_temp);
		i++;
	}
	fq.close();

	num_of_obs = obs.size();

}

void collisionDetection::print_obs(obst o) {
	cout << "x: " << o.x << ", y: " << o.y << ", radius: " <<  o.r << endl;
}

bool collisionDetection::check_collisions(Vector q, double robot_radius) {
	// Returns true if robot is not in collision with the obstacles

	if (num_of_obs == 0)
		return true;

	for (int i = 0; i < num_of_obs; i++) {

		if ( ((q[0]-obs[i].x)*(q[0]-obs[i].x) + (q[1]-obs[i].y)*(q[1]-obs[i].y)) <= (robot_radius + obs[i].r)*(robot_radius + obs[i].r) )
			return false;
	}

	return true;
}


/*
int main() {
	collisionDetection cd;

	Vector q = {-1.0059, 0.957, 0};

	cout << cd.check_collisions(q, 0.00001) << endl;

	return 0;
}*/


