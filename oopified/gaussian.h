/**
 * @file gaussian.h
 * @author Caitrin Eaton (ceaton@ncf.edu)
 * @brief Interface for a gaussian class in support of gaussian mixture models (GMM).
 * @version 0.1
 * @date 2022-03-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef Gaussian_H
#define Gaussian_H

#define _USE_MATH_DEFINES       // defines M_PI (double) and M_PIl (long)
#include <math.h>
#include <stdio.h>      /* printf, scanf, puts, NULL */

/**
 * @brief Gaussian representation using a mean, variance, and weight.
 */
class Gaussian
{
	public:
		explicit Gaussian( double mean_init=0.5, double variance_init=1.0, double weight_init=1.0/(2.0*M_PI) );
		
		// accessors
		double get_mean() const;
		double get_variance() const;
		double get_weight() const;
		double get_height( double x ) const;

		// mutators
		void set_mean( double mean_new );
		void set_variance( double variance_new );
		void set_weight( double weight_new );

		// utilities
		void display() const;

	private:
		double mean, variance, weight;
};

#endif