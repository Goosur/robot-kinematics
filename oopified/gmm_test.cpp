/**
 * @file gmm_test.cpp
 * @author Caitrin Eaton (ceaton@ncf.edu)
 * @brief Tests a GMM class in support of motion primitives.
 * @version 0.1
 * @date 2022-03-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "gmm.h"

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
 * @brief request a GMM's height at a randomly x coordinate
 * 
 * @param gmm pointer to a GMM object
 */
void test_height( GMM* gmm )
{
	double x = rand_double( 0.0, 1.0 );
	double y = gmm->get_height( x );
	printf( "\ng(%.3f) = %.3f\n", x, y );
}

/**
 * @brief test that mutators correctly alter private fields
 * 
 * @param gmm pointer to a Gaussian object
 */
void test_mutators( GMM* gmm )
{
	vector<double> means( gmm->get_k() );

	vector<double> variances( gmm->get_k() );
	vector<double> weights( gmm->get_k() );
	for( int i=0; i<gmm->get_k(); i++ )
	{
		means[i]     = rand_double( 0.0, 1.0 );
		variances[i] = rand_double( 0.0, 2.0 );
		weights[i]   = rand_double( -5.0, 5.0 );
	}
	gmm->set_means( means );
	gmm->set_variances( variances );
	gmm->set_weights( weights );
	gmm->set_offset( rand_double( 0.0, 2.0*M_PI ) );
	gmm->display();
}

/**
 * @brief test that accessors correctly read private fields
 * 
 * @param gmm pointer to a GMM object
 */
void test_accessors( GMM* gmm )
{
	printf( "\nTesting accessors of GMM object at location %p\n", gmm );
	vector<double> m = gmm->get_means();
	vector<double> v = gmm->get_variances();
	vector<double> w = gmm->get_weights();
	printf( "\tmeans     = %p\n", &m );
	printf( "\tvariances = %p\n", &v );
	printf( "\tweights   = %p\n", &w );
	printf( "\toffset    = %.3f\n", gmm->get_offset() );
}

int main()
{
	// seed random number generator with current timestamp
  	srand( time(NULL) );
	
	// create a gaussian and display its characteristics
	GMM* gmm = new GMM( 7 );
	gmm->display();
	test_height( gmm );

	// alter the gaussian
	test_mutators( gmm );
	test_height( gmm );

	// test accessors
	test_accessors( gmm );
	test_accessors( gmm );
	delete gmm;
	gmm = nullptr;

	// test the other constructor style
	vector<Gaussian> gaussians(3);
	gmm = new GMM( gaussians, rand_double( -10.0, 10.0 ));
	gmm->display();
	delete gmm;	
	gmm = nullptr;

	return 0;
}