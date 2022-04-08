#include "transform.h"
#include <iostream>
#include <math.h>

Transform::Transform(double alpha, double a, double d, double theta) : Matrix<double>(4, 4), alpha(alpha), a(a), d(d), theta(theta)
{
	this->A = vector<vector<double>>{
		vector<double>{cos(this->theta), -sin(this->theta), 0, this->a},
		vector<double>{sin(this->theta) * cos(this->alpha), cos(this->theta) * cos(this->alpha), -sin(this->alpha), -sin(this->alpha) * this->d},
		vector<double>{sin(this->theta) * sin(this->alpha), cos(this->theta) * sin(this->alpha), cos(this->alpha), cos(this->alpha) * this->d},
		vector<double>{0, 0, 0, 1}
	};
}


// ACCESSORS

double Transform::get_alpha() { return alpha; }

double Transform::get_a() { return a; }

double Transform::get_d() { return d; }

double Transform::get_theta() { return theta; }

vector<double> Transform::get_position() { return vector<double>{A[0][3], A[1][3], A[2][3]}; }


// MUTATORS

/**
 * @brief Update joint angle, theta
 * @param theta, double representing the joint angle, in radians
 */
void Transform::set_theta(double theta) { this->theta = theta; }

/**
 * @brief Update prismatic length, d
 * @param d, double representing the prismatic length, in cm
 */
void Transform::set_d(double d) { this->d = d; }


// UTILITIES
/** 
 * @brief Elementwise multiplication of each individual element with the same scalar value.
 * @param scalar, double by which to multiply each element of the matrix
 * @return Transform, the product of this Transform and the scalar multiplier
 */
Transform &Transform::operator*(double coefficient)
{
	for (int i = 0; i < this->A.size(); i++)
		for (int j = 0; j < this->A[0].size(); j++)
			this->A[i][j] = coefficient * this->A[i][j];

	return *this;
}

/** 
 * @brief Matrix multiplication
 * @param right, another Transform by which to matrix multiply using the dot product
 * @return Transform, a new Transform containing the product of the matrix multiplication this * right
 */
Transform &Transform::operator*(Transform right)
{
	Transform *temp = new Transform(0, 0, 0, 0);

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			temp->A[i][j] = 0;
			for (int k = 0; k < 4; k++)
			{
				temp->A[i][j] += this->A[i][k] * right.A[k][j];
			}
			
		}
	}		

	return *temp;
}

void Transform::to_string()
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
			std::cout << this->A[i][j] << "\t";
		std::cout << std::endl;
	}
}
