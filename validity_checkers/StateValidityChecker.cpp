/*
 * Checker.cpp
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

#include "StateValidityChecker.h"
#include <queue>

void StateValidityChecker::defaultSettings()
{
	cout << "here?\n";

	stateSpace_ = mysi_->getStateSpace().get();
	if (!stateSpace_)
		OMPL_ERROR("No state space for motion validator");
}

// ==========================================================================================================

void StateValidityChecker::retrieveStateVector(const ob::State *state, Vector &q) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

		// Set state of rod
		for (unsigned i = 0; i < n; i++) {
			q[i] = Q->values[i];
		}
}

void StateValidityChecker::updateStateVector(const ob::State *state, Vector q) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

	for (unsigned i = 0; i < n; i++) {
		Q->values[i] = q[i];
	}
}

void StateValidityChecker::printStateVector(const ob::State *state) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

	Vector q(n);

	for (unsigned i = 0; i < n; i++) {
		q[i] = Q->values[i]; // Set state of robot1
	}
	cout << "q: "; printVector(q);
}

// ==========================================================================================================

bool StateValidityChecker::isValid(const ob::State *state) {
	Vector q(n);
	retrieveStateVector(state, q);

	return check_collisions(q, robot_r);
}

bool StateValidityChecker::checkMotion(const ob::State *s1, const ob::State *s2)
{
	bool result = true;
	int nd = stateSpace_->validSegmentCount(s1, s2);

	/* initialize the queue of test positions */
	std::queue< std::pair<int, int> > pos;
	if (nd >= 2)
	{
		pos.push(std::make_pair(1, nd - 1));

		/* temporary storage for the checked state */
		ob::State *test = mysi_->allocState();

		/* repeatedly subdivide the path segment in the middle (and check the middle) */
		while (!pos.empty())
		{
			std::pair<int, int> x = pos.front();

			int mid = (x.first + x.second) / 2;
			stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, test);

			if (!isValid(test))
			{
				result = false;
				break;
			}

			pos.pop();

			if (x.first < mid)
				pos.push(std::make_pair(x.first, mid - 1));
			if (x.second > mid)
				pos.push(std::make_pair(mid + 1, x.second));
		}

		mysi_->freeState(test);
	}

	return result;
}

// ========================== Reconstruct ===========================================================

bool StateValidityChecker::reconstructMotion(const ob::State *s1, const ob::State *s2, Matrix &M) {

	int nd = stateSpace_->validSegmentCount(s1, s2);

	/* temporary storage for the checked state */
	ob::State *test = mysi_->allocState();

	Vector q(n);

	for (int i = 0; i <= nd; i++) {
		stateSpace_->interpolate(s1, s2, (double)i / (double)nd, test);
		retrieveStateVector(test, q);

		M.push_back(q);
	}

	return true;
}



// ========================== Misc ==================================================================

template <class T>
void StateValidityChecker::printVector(T q) {

	cout << "[";
	for (int i = 0; i < q.size(); i++)
		cout << q[i] << " ";
	cout << "]" << endl;

}
void StateValidityChecker::printMatrix(Matrix M) {
	for (unsigned i = 0; i < M.size(); i++) {
		for (unsigned j = 0; j < M[i].size(); j++)
			cout << M[i][j] << " ";
		cout << endl;
	}
}

double StateValidityChecker::normDistance(Vector a1, Vector a2, int d) {
	if (d==-1)
		d = a1.size();

	double sum = 0;
	for (int i=0; i < d; i++)
		sum += (a1[i]-a2[i])*(a1[i]-a2[i]);
	return sqrt(sum);
}

// ========================== Two-wheels motion ===========================================================

bool StateValidityChecker::checkMotionTW(const ob::State *s1, const ob::State *s2) {

	Vector q1(n), q2(n), q(n);
	retrieveStateVector(s1, q1);
	retrieveStateVector(s2, q2);

	//if (is_Goal_node) { // Add move straight to goal if the goal is sampled
	//}
	//else
	if (!GetShortestPath(q1, q2))
		return false;

	cout << "v: "; printVector(v);
	cout << "w: "; printVector(w);
	cout << "t: "; printVector(t);

	return false;

	Matrix Q;
	Q.push_back(q1);

	for (int i = 0; i < v.size(); i++) {
		int m = 1+ceil(t[i]/dt); // dt + j*dt ?
		double dd = t[i] / (m-1);
		q = Q.back();
		for (int j = 1; j < m; j++) // starts from 1 because the first point was already inserted to Q
			Q.push_back(myprop(q, v[i], w[i], j*dd));
	}

	cout << "------\n";
	printMatrix(Q);
}

Vector StateValidityChecker::myprop(Vector q, double vi, double wi, double ti) {
double h = q[2] + wi * ti;
double x, y;

if (wi) {
    x = q[0] + ((vi/wi)*(sin(h)-sin(q[2])));
    y = q[1] - ((vi/wi)*(cos(h)-cos(q[2])));
}
else {
    x = q[0] + ti * vi * cos(h);
    y = q[1] + ti * vi * sin(h);
}

return {x, y, h};

}

bool StateValidityChecker::GetShortestPath(Vector q1, Vector q2) {

    //   start and goal configurations:
	//       q1 = [x;y;theta]
	//       q2 = [x;y;theta]
	//
	//   minimum turning radius:
	//       turn_radius
	//
	//   shortest path (all 1x3 matrices):
	//       v is forward speed (normalized - always 1 or -1)
	//       w is turning rate
	//       t is time interval (i.e., apply v(i) and w(i) for time t(i))
	//
	//   if no path is found, [v,w,t] will all be empty sets
	//
	//   note that the total length of the shortest path is sum(t)

	double d1 = 1e9, dmid = 1e9, d2 = 1e9;
	short s1, s2, dir;
	bool sol = false;

	for (int s1cur = -1; s1cur < 2; s1cur+=2)
		for (int s2cur = -1; s2cur < 2; s2cur+=2) {
			for (int dircur = -1; dircur < 2; dircur+=2) {
				bool kt = twb_shortpath(q1, s1cur, q2, s2cur, dircur);
				cout << s1cur << " " << s2cur << " " << dircur << endl;
				cout << kt << endl;
				if (kt) {
					printVector(d1mid2);
					if (d1mid2[0]+d1mid2[1]+d1mid2[2] < d1+dmid+d2) {
						sol = true;

						t = d1mid2;
						s1 = s1cur;
						s2 = s2cur;
						dir = dircur;
					}
				}

			}
		}

	if (!sol)
		return false;

	v = {dir, dir, dir};
	w = {s1*v[0]/turn_radius, 0, s2*v[2]/turn_radius};

	return true;
}

bool StateValidityChecker::twb_shortpath(Vector q1, short s1, Vector q2, short s2, short dir) {

	//   q1,q2 are initial and final points
	//   s1,s2 are -1 for wheels to left, +1 for wheels to right
	//   dir is 1 for forward and -1 for backward
	//   r is turning radius
	//
	//   d1,dmid,d2 are lengths of each part of the curve in R^2
	//   qt1,qt2 are intermediate points
	//
	//   NOTE: particular combinations of dir, s1, s2 are sometimes infeasible.
	//   If this is the case, false is returned.

	double h1 = q1[2];
	double h2 = q2[2];
	qc1[0] = q1[0] - turn_radius * s1 * sin(h1);
	qc1[1] = q1[1] + turn_radius * s1 * cos(h1);
	qc2[0] = q2[0] - turn_radius * s2 * sin(h2);
	qc2[1] = q2[1] + turn_radius * s2 * cos(h2);

	if (s1*s2<0 && normDistance(qc1, qc2) < 2*turn_radius)
		return false;

	twb_gettangent(qc1, dir*s1, qc2, dir*s2);

	double delta1 = twb_getangle(qc1,qt1) - twb_getangle(qc1,q1);
	if ( dir*s1 > 0 && delta1 < 0 )
	    delta1 += 2*PI;
	else if ( dir*s1 < 0 && delta1 > 0 )
	    delta1 -= 2*PI;

	double delta2 = twb_getangle(qc2,q2)-twb_getangle(qc2,qt2);
	if ( dir*s2 > 0 && delta2 < 0 )
		delta2 += 2*PI;
	else if ( dir*s2 < 0 && delta2 > 0 )
		delta2 -= 2*PI;

	d1mid2 = {fabs(turn_radius*delta1), normDistance(qt1, qt2, 2), fabs(turn_radius*delta2)};
	qt1[2] = h1 + delta1;
	qt2[2] = h2 + delta2;

	return true;
}

void StateValidityChecker::twb_gettangent(Vector q1,short s1, Vector q2, short s2) {
	// updates qt1 and qt2

	// q1,q2 are the centers of two circles, both with radius r
	// s1,s2 are the orientations (1 or -1) of the two circles
	//		(1=cc-wise, -1=c-wise)
	//
	// t1,t2 are the endpoints of a segment that is tangent to
	//		both circles and that matches the orientations s1,s2

	if ( s1==1 && s2==1 ) {
		// Vector u from center 1 to center 2
		double ux = q2[0] - q1[0];
		double uy = q2[1] - q1[1];
		double norm_u = sqrt(ux*ux + uy*uy);

		// Vector perpendicular to u (turning clockwise) of length r
		//double uperp_x = turn_radius * uy / norm_u;
		//double uperp_y = -turn_radius * ux / norm_u;

		// Tangent points
		qt1[0] = q1[0] + turn_radius * uy / norm_u;
		qt1[1] = q1[1] - turn_radius * ux / norm_u;
		qt2[0] = q2[0] + turn_radius * uy / norm_u;
		qt2[1] = q2[1] - turn_radius * ux / norm_u;
	}
	else if ( s1==-1 && s2==-1 ) {
		// Vector u from center 1 to center 2
		double ux = q2[0] - q1[0];
		double uy = q2[1] - q1[1];
		double norm_u = sqrt(ux*ux + uy*uy);

		// Vector perpendicular to u (turning counter-clockwise) of length r
		//double uperp_x = -turn_radius * uy / norm_u;
		//double uperp_y = turn_radius * ux / norm_u;

		// Tangent points
		qt1[0] = q1[0] - turn_radius * uy / norm_u;
		qt1[1] = q1[1] + turn_radius * ux / norm_u;
		qt2[0] = q2[0] - turn_radius * uy / norm_u;
		qt2[1] = q2[1] + turn_radius * ux / norm_u;
	}
	else if ( s1==1 && s2==-1 ) {
		// Vector to midpoint of segment from center 1 to center 2
		double qmid_x = (q2[0]-q1[0])/2; // !!! Should it be (+) instead of (-) ???
		double qmid_y = (q2[1]-q1[1])/2;

		// Vector to tangent points in local coordinates
		double L = sqrt(qmid_x*qmid_x + qmid_y*qmid_y);
		double x = (turn_radius*turn_radius)/L;
		double y = (turn_radius/L)*sqrt((L*L)-(turn_radius*turn_radius));

		// Unit vector in the direction of qmid, and its perpendicular
		qmid_x /= L;
		qmid_y /= L;

		// Tangent points
		qt1[0] = q1[0] + x * qmid_x + y * qmid_y;
		qt1[1] = q1[1] + x * qmid_y - y * qmid_x;
		qt2[0] = q2[0] - x * qmid_x - y * qmid_y;
		qt2[1] = q2[1] - x * qmid_y + y * qmid_x;
	}
	else if ( s1==-1 && s2==1 ) {
		// Vector to midpoint of segment from center 1 to center 2
		double qmid_x = (q2[0]-q1[0])/2; // !!! Should it be (+) instead of (-) ???
		double qmid_y = (q2[1]-q1[1])/2;

		// Vector to tangent points in local coordinates
		double L = sqrt(qmid_x*qmid_x + qmid_y*qmid_y);
		double x = (turn_radius*turn_radius)/L;
		double y = (turn_radius/L)*sqrt((L*L)-(turn_radius*turn_radius));

		// Unit vector in the direction of qmid, and its perpendicular
		qmid_x /= L;
		qmid_y /= L;

		// Tangent points
		qt1[0] = q1[0] + x * qmid_x + y * qmid_y;
		qt1[1] = q1[1] + x * qmid_y - y * qmid_x;
		qt2[0] = q2[0] - x * qmid_x - y * qmid_y;
		qt2[1] = q2[1] - x * qmid_y + y * qmid_x;
	}
	else {
		printf("Error: twb_gettangent was passed s1=%d, s2=%d",s1,s2);
		exit(1);
	}
}

double StateValidityChecker::twb_getangle(Vector q1, Vector q2) {

	return atan2(q2[1]-q1[1],q2[0]-q1[0]);

}









