/*
 * SnakeSimDefines.h
 *
 *  Created on: Jul 6, 2016
 *      Author: arman
 */

#ifndef INCLUDE_SNAKESIMDEFINES_H_
#define INCLUDE_SNAKESIMDEFINES_H_

//#define USE_IRR
#define USE_DEM


#ifdef USE_DEM
#define CHSYSTEM ChSystemDEM
#else
#define CHSYSTEM ChSystem
#endif


#endif /* INCLUDE_SNAKESIMDEFINES_H_ */
