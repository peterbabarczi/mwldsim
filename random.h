/** 
 \file random.h
 \brief Random generation.
 
 \author              Soproni Peter
 \author              soproni@tmit.bme.hu
 \date                2012 april
*/

#pragma once
#ifndef RANDOM_H
#define RANDOM_H

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>


class BRandom
{
public:
	static BRandom* instance();
	// If this is set the used seed is varied with every initialization based on the current time
	static bool timeBasedSeed; 

	double GetNextUniformDouble(double _upperbound = 1, double _lowerbound = 0);
	double GetNextEponencialDouble(double _lambda);
	int GetNextUniformInt(int _upperbound = 100, int _lowerbound = 0);

private:
	static void atExit();
	static BRandom* rand; 

	BRandom(int seed);
	BRandom();
	boost::variate_generator<boost::minstd_rand, boost::uniform_real<> > generator;  
};

#endif

