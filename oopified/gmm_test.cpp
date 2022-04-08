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
double rand_double(double min, double max)
{
	return ((double)rand() / RAND_MAX) * (max - min) + min;
}

/**
 * @brief request a GMM's height at a randomly x coordinate
 * 
 * @param gmm pointer to a GMM object
 */
void test_height(GMM* gmm)
{
	double x = rand_double(0.0, 1.0);
	double y = gmm->at(x);
	printf("\ngmm(%.3f) = %.3f\n", x, y);
}

/**
 * @brief test that mutators correctly alter private fields
 * 
 * @param gmm pointer to a Gaussian object
 */
void test_mutators(GMM* gmm)
{
	vector<double> means(gmm->get_k());
	vector<double> stds(gmm->get_k());
	vector<double> weights( gmm->get_k() );
	for(int i = 0; i < gmm->get_k(); i++)
	{
		weights[i] = rand_double(-5.0, 5.0);
		means[i] = rand_double(0.0, 1.0);
		stds[i] = rand_double(0.0, 2.0);
	}
	gmm->set_weights(weights);
	gmm->set_means(means);
	gmm->set_stds(stds);
	gmm->set_offset(rand_double(0.0, 2.0*M_PI));
	gmm->to_string();
}

/**
 * @brief test that accessors correctly read private fields
 * 
 * @param gmm pointer to a GMM object
 */
void test_accessors(GMM* gmm)
{
	printf("\nTesting accessors of GMM object at location %p\n", gmm);
	vector<double> w = gmm->get_weights();
	vector<double> m = gmm->get_means();
	vector<double> s = gmm->get_stds();
	printf("\tweights\t= %p\n", &w);
	printf("\tmeans\t= %p\n", &m);
	printf("\tstds\t= %p\n", &s);
	printf("\toffset\t= %.3f\n", gmm->get_offset());
}

int main()
{
	// seed random number generator with current timestamp
  	srand(time(NULL));
	
	// create a gaussian and display its characteristics
	GMM* gmm = new GMM(7);
	gmm->to_string();
	test_height(gmm);

	// alter the gaussian
	test_mutators(gmm);
	test_height(gmm);

	// test accessors
	test_accessors(gmm);
	delete gmm;
	gmm = nullptr;

	// test the other constructor style
	vector<Gaussian> gaussians(3);
	gmm = new GMM(gaussians, rand_double(-10.0, 10.0));
	gmm->to_string();
	delete gmm;	
	gmm = nullptr;

	return 0;
}
