#ifndef GAUSSIAN_H
#define GAUSSIAN_H

#include <math.h>
#include <stdio.h>
#include "BasisFunction.h"

class Gaussian : public BasisFunction
{
	public:
		explicit Gaussian(double weight=1.0, double mean=0.0, double std=1.0) : weight(weight), mean(mean), std(std) {}

		double at(double x) override { return this->weight * exp(-pow(x - this->mean, 2) / (2 * pow(this->std, 2))); }
	  
		// accessors
    double get_weight() const { return this->weight; }
		double get_mean() const { return this->mean; }
		double get_std() const { return this->std; }

		// mutators
		void set_weight(double weight) { this->weight = weight; }
		void set_mean(double mean) { this->mean = mean; }
		void set_std(double std) { this->std = std; }

    // to_string
    void to_string() override
    {
      printf("\nGaussian object at location %p\n", this);
      printf("\tweight\t= %.3f", this->weight);
      printf("\tmean\t= %.3f", this->mean);
      printf("\tstd\t= %.3f", this->std);
      printf("\tg(x)\t= %.3f * exp(-(x - %.3f)^2 / (2 * %.3f^2))\n", this->weight, this->mean, this->std);
    }

	private:
    double weight;
    double mean;
    double std;
};

#endif
