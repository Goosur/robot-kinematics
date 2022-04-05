/**
 * @file gaussian.cpp
 * @author Caitrin Eaton (ceaton@ncf.edu)
 * @brief Implements a gaussian class in support of gaussian mixture models (GMM).
 * @version 0.1
 * @date 2022-03-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "gaussian.h"

Gaussian::Gaussian( double mean_init, double variance_init, double weight_init )
	: mean{ mean_init }, variance{ variance_init }, weight{ weight_init }
{
}


// ACCESSORS

/** @brief return the mean */
double Gaussian::get_mean() const
{
	return mean;
}

/** @brief return the variance */
double Gaussian::get_variance() const
{
	return variance;
}

/** @brief return the weight */
double Gaussian::get_weight() const
{
	return weight;
}

/**
 *  @brief return the y-value for a given x coordinate
 *  @param x (double) point along the horizontal axis to plug into the gaussian equation
 *  @return y (double) height of the gaussian at the given x-coordinate
 */
double Gaussian::get_height( double x ) const
{
	return weight * exp( -0.5 * pow(x-mean, 2) / variance );
}


// MUTATORS

/** @brief change the mean */
void Gaussian::set_mean( double mean_new )
{
	mean = mean_new;
}

/** @brief change the variance */
void Gaussian::set_variance( double variance_new )
{
	variance = variance_new;
}

/** @brief change the weight */
void Gaussian::set_weight( double weight_new )
{
	weight = weight_new;
}


// UTILITIES

/** @brief display gaussian characteristics in the terminal */
void Gaussian::display() const
{
	printf( "\nGaussian object at location %p\n", this );
	printf( "\tmean     = %.3f\n", mean );
	printf( "\tvariance = %.3f\n", variance );
	printf( "\tweight   = %.3f\n", weight );
	printf( "\tg(x)     = %.3f * exp( -(x - %.3f)^2 / (2 * %.3f) )\n", weight, mean, variance );
}