/**
 * @file gmm.h
 * @author Caitrin Eaton (ceaton@ncf.edu)
 * @brief Interface for a Gaussian Mixture Model (GMM) class in support of motion primitives.
 * @version 0.1
 * @date 2022-03-05
 * 
 * @copyright Copyright (c) 2022 
 */
#ifndef GMM_H
#define GMM_H

#include <string>
#include <vector>
#include "gaussian.h"

using std::vector;
using std::string;

/**
 * @brief GMM representation using vector of gaussians.
 * 
 * TODO:
 * 		1. construct from input filepath: GMM( in_filepath )?
 * 		2. write( output_file_stream )?
 * 		3. string field for name of task?
 * 		4. string field for name of joint?
 */
class GMM
{
	public:
		// constructors
		explicit GMM( int k=5 );
		GMM( vector<Gaussian> gaussians, double offset );

		// constants
		const char CH_SIGMA = 228;
		
		// accessors
		vector<Gaussian> get_gaussians() const;
		Gaussian get_gaussian( int i ) const;
		double get_height( double x ) const;
		vector<double> get_means() const;
		vector<double> get_variances() const;
		vector<double> get_weights() const;
		double get_offset() const;
		int get_k() const;

		// mutators
		void set_means( const vector<double> & m );
		void set_variances( const vector<double> & v );
		void set_weights( const vector<double> & w );
		void set_offset( double y );

		// utilities
		void display() const;


	private:
		vector<Gaussian> gaussians;
		double offset;
		int k;
};

#endif