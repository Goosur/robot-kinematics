/**
 * @file gmm.cpp
 * @author Caitrin Eaton (ceaton@ncf.edu)
 * @brief Implements a GMM class in support of motion primitives.
 * @version 0.1
 * @date 2022-03-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "gmm.h"

GMM::GMM( int k )
{
	this->k = k;
	this->gaussians.resize(k);
	this->offset = 0;

	// distribute gaussians across the X axis, range [0.0, 1.0]
	double dx = 1.0 / ((double)k - 1.0);
	double mean = 0.0;
	double variance = dx*dx;
	double weight = 1.0 / pow(2.0 * M_PI * variance, 0.5 );
	for( int i=0; i < k; i++ )
	{
		gaussians[i].set_mean( mean ); // = new Gaussian( mean, variance, weight );
		gaussians[i].set_variance( variance );
		gaussians[i].set_weight( weight );
		mean += dx;
	}
}

GMM::GMM( vector<Gaussian> gaussians, double offset )
{
	this->gaussians = gaussians;
	this->offset = offset;
	this->k = gaussians.size();
}


// ACCESSORS

/** @brief return ,a pointer to the vector of all k gaussian kernels 
*	@return vector<Gaussian> of all k gaussian kernels
*/
vector<Gaussian> GMM::get_gaussians() const
{
	return gaussians;
}

/** @brief return a pointer to the i-th gaussian kernel
*   @param i (int), index of a specific kernel within the GMM
*	@return Gaussian at index i in the GMM
*/
Gaussian GMM::get_gaussian( int i ) const
{
	return gaussians[i];
}

/** @brief return a vector of all k gaussian kernels' means 
*	@return vector<double> means of all gaussian kernels in the GMM
*/
vector<double> GMM::get_means() const
{
	vector<double> means(k);
	for( int i=0; i<k; i++ )
	{
		means[i] = gaussians[i].get_mean();
	}
	return means;
}

/** @brief return a vector of all k gaussian kernels' variances
*	@return vector<double> variances of all k gaussian kernels
*/
vector<double> GMM::get_variances() const
{
	vector<double> variances(k);
	for( int i=0; i<k; i++ )
	{
		variances[i] = gaussians[i].get_variance();
	}
	return variances;
}

/** @brief return a vector of all k gaussian kernels' weights 
*	@return vector<double> weights of all k gaussian kernels
*/
vector<double> GMM::get_weights() const
{
	vector<double> weights(k);
	for( int i=0; i<k; i++ )
	{
		weights[i] = gaussians[i].get_weight();
	}
	return weights;
}

/**
 * @brief return the offset
 * @return double, y-intercept of the GMM model
 */
double GMM::get_offset() const
{
	return offset;
}

/**
 *  @brief return the y-value for a given x coordinate
 *  @param x (double) point along the horizontal axis to plug into the gaussian equation
 *  @return y (double) height of the gaussian at the given x-coordinate
 */
double GMM::get_height( double x ) const
{
	double y = offset;
	for( int i=0; i<k; i++ )
	{
		y += gaussians[i].get_height( x );
	}
	return y;
}

/**
 * @brief return the number of gaussian kernels in the model
 * @return k (int) the number of gaussian kernels in the model
 */
int GMM::get_k() const
{
	return k;
}


// MUTATORS

/** @brief change the means 
 *  @param means, reference to a vector<double> of the gaussian kernels' new centroids
 */
void GMM::set_means( const vector<double> & means )
{
	for( int i=0; i<k; i++ )
	{
		gaussians[i].set_mean( means[i] );
	}	
}

/** @brief change the variances 
 *  @param variances, reference to a vector<double> of the gaussian kernels' new widths
 */
void GMM::set_variances( const vector<double> & variances )
{
	for( int i=0; i<k; i++ )
	{
		gaussians[i].set_variance( variances[i] );
	}	
}

/** @brief change the weights 
 *  @param weights, reference to a vector<double> of the gaussian kernels' new heights
 */
void GMM::set_weights( const vector<double> & weights )
{
	for( int i=0; i<k; i++ )
	{
		gaussians[i].set_weight( weights[i] );
	}	
}

/** @brief change the y-intercept 
 *  @param y (double) the GMM's new y-intercept 
 */
void GMM::set_offset( double y )
{
	offset = y;
}


// UTILITIES

/** @brief display gaussian characteristics in the terminal */
void GMM::display() const
{
	string mean_str = "\tmeans     =  [ ";
	string var_str  = "\tvariances =  [ ";
	string w_str    = "\tweights   =  [ ";
	
	for( int i=0; i<k; i++ )
	{
		mean_str += std::to_string( gaussians[i].get_mean() );
		var_str += std::to_string( gaussians[i].get_variance() );
		w_str += std::to_string( gaussians[i].get_weight() );

		if( i+1 < k ){
			mean_str += ", ";
			var_str += ", ";
			w_str += ", ";
		} else {
			mean_str += " ]\n";
			var_str += " ]\n";
			w_str += " ]\n";
		}

	}	

	printf( "\nGMM object at location %p\n", this );
	printf( "%s", mean_str.c_str() );
	printf( "%s", var_str.c_str() );
	printf( "%s", w_str.c_str() );
	printf( "\toffset  =    %.3f\n", offset );
	printf( "\tgmm(x)  =    %.3f + Σ[i=1:%d]( w[i] * exp( -(x - mean[i])^2 / (2 * variance[i]) )\n", offset, k );
}