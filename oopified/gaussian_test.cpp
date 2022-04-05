/**
 * @file gaussian_test.cpp
 * @author Caitrin Eaton (ceaton@ncf.edu)
 * @brief Tests a gaussian class in support of gaussian mixture models (GMM).
 * @version 0.1
 * @date 2022-03-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "gaussian.h"

/**
 * @brief generate a random double within a specified range
 * 
 * @param min (double) lower bound
 * @param max (double) upper bound
 * @return double between min and max
 */
double rand_double( double min, double max )
{
	return ((double)rand() / RAND_MAX) * (max - min) + min;
}

/**
 * @brief request a Gaussian's height at a randomly x coordinate
 * 
 * @param g pointer to a Gaussian object
 */
void test_height( Gaussian* g )
{
	double x = rand_double( 0.0, 1.0 );
	double y = g->get_height( x );
	printf( "\ng(%.3f) = %.3f\n", x, y );
}

/**
 * @brief test that mutators correctly alter private fields
 * 
 * @param g pointer to a Gaussian object
 */
void test_mutators( Gaussian* g )
{
	double mean = rand_double( 0.0, 1.0 );
	double variance = rand_double( 0.0, 2.0 );
	double weight = rand_double( -5.0, 5.0 );
	g->set_mean( mean );
	g->set_variance( variance );
	g->set_weight( weight );
	g->display();
}

/**
 * @brief test that accessors correctly read private fields
 * 
 * @param g pointer to a Gaussian object
 */
void test_accessors( Gaussian* g )
{
	printf( "\nTesting accessors of Gaussian object at location %p\n", g );
	double m = g->get_mean();
	double v = g->get_variance();
	double w = g->get_weight();
	printf( "\tmean     = %.3f\n", m );
	printf( "\tvariance = %.3f\n", v );
	printf( "\tweight   = %.3f\n", w );
	printf( "\tg(x)     = %.3f * exp( -(x - %.3f)^2 / (2 * %.3f) )\n", w, m, v );
}

int main()
{
	// seed random number generator with current timestamp
  	srand( time(NULL) );
	
	// create a gaussian and display its characteristics
	Gaussian* g = new Gaussian();
	g->display();
	test_height( g );

	// alter the gaussian
	test_mutators( g );
	test_accessors( g );
	test_height( g );

	return 0;
}