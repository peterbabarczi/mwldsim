/** 
 \file random.cpp
 \brief Random generation.
 
 \author              Soproni Peter
 \author              soproni@tmit.bme.hu
 \date                2012 april
*/

#include "random.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <ctime>
#include <math.h>
#include <iostream>

double BRandom::GetNextUniformDouble(double _upperbound, double _lowerbound)
{
	if  (_upperbound < _lowerbound)
	{
		double temp = _lowerbound;
		_lowerbound = _upperbound;
		_upperbound = temp;
	}
	double ret = generator();
	ret *=_upperbound-_lowerbound;
	ret +=_lowerbound;
	return ret;
}

double BRandom::GetNextEponencialDouble(double _lambda)
{
	if (_lambda > 0)
		return -1*log(GetNextUniformDouble())*_lambda;

	return log(GetNextUniformDouble())*_lambda;
}

int BRandom::GetNextUniformInt(int _upperbound , int _lowerbound)
{
	if (_upperbound < _lowerbound)
	{
		int temp = _lowerbound;
		_lowerbound = _upperbound;
		_upperbound = temp;
	}
	int ret = (int)GetNextUniformDouble(_upperbound+1,_lowerbound);
	if (ret > _upperbound)
		ret = _lowerbound;
	if (ret < _lowerbound)
		ret = _upperbound;
	return ret;
}

BRandom::BRandom() : generator(boost::minstd_rand(), boost::uniform_real<>(0,1))
{
}

BRandom::BRandom(int seed) : generator(boost::minstd_rand(((seed == 0) ? -1 : seed)), boost::uniform_real<>(0,1))
{
}

// To generate a solution for generating and deleting Randoms without memory leak
void BRandom::atExit()
{
	if (rand == NULL)
		return;

	delete rand;
	rand = NULL;
}

BRandom* BRandom::instance()
{
	if (rand == NULL)
	{
		std::atexit(atExit);
		if (timeBasedSeed)
			rand = new BRandom(time(NULL));
		else
			rand = new BRandom();
	}
	return rand;
}

BRandom* BRandom::rand = NULL;
bool BRandom::timeBasedSeed = false;
