/**
 * @file Transform.h
 * @author Caitrin Eaton (ceaton@ncf.edu)
 * @brief Interface for a transformation matrix class in support if forward kinematics. Built 
 *          on top of Weiss' Matrix class, with the constraint that all elements are doubles,
 *          and the addition of a matrix multiplication operation (*).
 * @version 0.1
 * @date 2022-03-05
 * 
 * @copyright Copyright (c) 2022
 */

 #include "matrix.h"

class Transform : private Matrix
{
	public:
		// constructor
		Transform( double alpha=0.0, double a=0.0, double d=0.0, double theta=0.0 );
		
		// accessors
		double get_alpha();
		double get_a();
		double get_d();
		double get_theta();
		vector<double> get_position();
		// TODO: double get_roll();
		// TODO: double get_pitch();
		// TODO: double get_yaw();
		// TODO: vector<double> get_orientation(); // returns {roll, pitch, yaw}

		// mutators
		void set_theta( double theta );
		void set_d( double d );

		// utilities
		void display();
		const vector<vector<Object>> & operator*( double coefficient ) const;
		vector<Object> & operator*( double coefficient );
		const vector<Object> & operator*( Transform m ) const;
		vector<Object> & operator*( Transform m );

	private:
		double alpha, a, d, theta;
};