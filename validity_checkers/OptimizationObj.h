/*
 * OptimizationObj.h
 *
 *  Created on: Aug 22, 2017
 *      Author: avishai
 */

#ifndef VALIDITY_CHECKERS_OPTIMIZATIONOBJ_H_
#define VALIDITY_CHECKERS_OPTIMIZATIONOBJ_H_

#include "ompl/base/Cost.h"

class OptimizationObj
{
public:
	OptimizationObj();

	ob::Cost motionCost(Matrix);

	ob::Cost pathLengthObjective(Matrix);

	/** \brief Get the value of the cost */
	ob::Cost get_mCost() {
		return mCost;
	}
private:
	/** \brief Local connection cost between two states */
	ob::Cost mCost;

};




#endif /* VALIDITY_CHECKERS_OPTIMIZATIONOBJ_H_ */
