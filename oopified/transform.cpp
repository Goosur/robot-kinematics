

#include "transform.h"

Transform::Transform( double alpha, double a, double d, double theta )
{
	this = Matrix<double>( 4, 4 );
	this->alpha = alpha;
	this->a 	= a;
	this->d 	= d;
	this->theta = theta;
}

// ACCESSORS

double Transform::get_alpha()
{
	return alpha;
}

double Transform::get_a()
{
	return a;
}

double Transform::get_d()
{
	return d;
}

double Transform::get_theta()
{
	return theta;
}

vector<double> Transform::get_position()
{
	vector<double> xyz = {array[0][3], array[1][3], array[2][3]};
	return xyz;
}


// MUTATORS

/**
 * @brief Update joint angle, theta
 * @param theta, double representing the joint angle, in radians
 */
void Transform::set_theta( double theta )
{
	this->theta = theta;
}

/**
 * @brief Update prismatic length, d
 * @param d, double representing the prismatic length, in cm
 */
void Transform::set_d( double d )
{
	this->d = d;
}


// UTILITIES

/** @brief Pretty-print the transformation matrix to the terminal */
void Transform::display()
{
	string arrStr;
	string buffer;
	sprintf( arrStr, "Transform @ %p, theta = %.3f\n", this, theta );
	for( int row=0; row < array.size(); row++ ){
		arrStr += "\t";
		for( int col=0; col < array[row].size(); col++ ){
			sprintf( buffer, "%.3f\t", array[row][col] ); 
			arrStr += buffer;
		}
		arrStr += "\n";
	}
	printf( arrStr )
	
}

/** 
 * @brief Elementwise multiplication of each individual element with the same scalar value.
 * @param scalar, double by which to multiply each element of the matrix
 * @return Transform, the product of this Transform and the scalar multiplier
 */
const Transform & Transform::operator*( double scalar ) const;

/** 
 * @brief Elementwise multiplication of each individual element with the same scalar value.
 * @param scalar, double by which to multiply each element of the matrix
 * @return Transform, the product of this Transform and the scalar multiplier
 */
vector<Object> & Transform::operator*( double coefficient );

/** 
 * @brief Matrix multiplication
 * @param right, another Transform by which to matrix multiply using the dot product
 * @return Transform, a new Transform containing the product of the matrix multiplication this * right
 */
const vector<Object> & Transform::operator*( Transform right ) const;

/** 
 * @brief Matrix multiplication
 * @param right, another Transform by which to matrix multiply using the dot product
 * @return Transform, a new Transform containing the product of the matrix multiplication this * right
 */
vector<Object> & Transform::operator*( Transform right );