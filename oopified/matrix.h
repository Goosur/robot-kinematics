/**
 * @file matrix.h
 * @author Mark Allen Weiss
 * @author Caitrin Eaton
 * @brief Implements the Matrix class from "Data Structurea and Algorithms in C++" (2014), p. 45.
 * 			Modified so that the formerly private "array" field can be accessed in child classes.
 * @version 0.1
 * @date 2022-03-05
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
using std::vector;

template <typename T>
class Matrix
{
	public:
		Matrix(int rows, int cols) : A(rows)
		{
			for(auto &row: A)
				row.resize(cols);
		}

		Matrix(vector<vector<T>> v) : A(v) {}
		Matrix(vector<vector<T>> &&v) : A(std::move(v)) {}

		vector<T> &operator[](int row) { return A[row]; }
		
		int numrows() const { return A.size(); }
		int numcols() const { return numrows() ? A[0].size() : 0; }
		
	protected:	// Modified (from "private") to support inheritance
		vector<vector<T>> A;
};
#endif
