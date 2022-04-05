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

template <typename Object>
class Matrix
{
	public:
		Matrix( int rows, int cols ) : array( rows )
		{
			for( auto & thisRow : array )
				thisRow.resize( cols );
		}

		Matrix( vector<vector<Object>> v ) : array( v ){ }
		Matrix( vector<vector<Object>> && v ) : array( std::move( v ) ){ }

		const vector<Object> & operator[]( int row ) const
			{ return array[ row ]; }
		vector<Object> & operator[]( int row )
			{ return array[ row ]; }
		
		int numrows() const
			{ return array.size(); }
		int numcols() const
			{ return numrows() ? array[ 0 ].size() : 0; }

		
	protected:	// Modified (from "private") to support inheritance
		vector<vector<Object>> array;
};
#endif