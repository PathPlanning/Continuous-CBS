/*
This file is part of C++lex, a project by Tommaso Urli.
C++lex is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
C++lex is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with C++lex.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "pilal.h"
#include <utility>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>

using std::pair;
using std::make_pair;
using std::swap;

namespace pilal {

	/*
	Matrix
	*/

	Matrix::Matrix() :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false),
		values(new storage(0)),
		rows(0),
		columns(0) {

	}

	Matrix::Matrix(char const * values) :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false) {

		int chunks = 0;
		long double ignore;
		std::stringstream buffer(values);
		while (!buffer.eof()) {
			buffer >> ignore;
			++chunks;
		}

		this->rows = 1;
		this->columns = chunks;
		this->values = new storage(chunks);
		set_values(values);

	}

	Matrix::Matrix(int n) :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false),
		values(new storage(n * n)),
		rows(n),
		columns(n) {
	}

	Matrix::Matrix(int n, long double v) :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false),
		values(new storage(n * n, v)),
		rows(n),
		columns(n) {
	}

	Matrix::Matrix(int r, int c) :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false),
		values(new storage(r * c)),
		rows(r),
		columns(c) {
	}

	Matrix::Matrix(int r, int c, long double v) :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false),
		values(new storage(r * c, v)),
		rows(r),
		columns(c) {
	}

	Matrix::Matrix(Matrix const& m) :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false),
		values(new storage(*m.values)),
		rows(m.rows),
		columns(m.columns) {

	}

	Matrix::Matrix(AnonymousMatrix m) :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false),
		values(new storage(0)),
		rows(m.rows),
		columns(m.columns) {

		swap(values->contents, m.values->contents);
	}


	Matrix::~Matrix() {
		delete values;
	}


	// Operators overloading with storage accessor
	long double& Matrix::operator() (int r, int c) {
		return values->at(r * columns + c);
	}

	long double const& Matrix::operator() (int r, int c) const {
		return values->at(r * columns + c);
	}

	// Operators overloading with storage accessor
	long double& Matrix::operator() (int i) {
		if (rows == 1 || columns == 1)
			return values->at(i);
		else
			throw(NotAVectorException());
	}

	long double const& Matrix::operator() (int i) const {
		if (rows == 1 || columns == 1)
			return values->at(i);
		else
			throw(NotAVectorException());
	}

	void Matrix::set_row(int i, char const* row) {

		std::stringstream buffer(row);

		for (int j = 0; j < columns; ++j)
			buffer >> at(i, j);
	}

	void Matrix::set_column(int j, char const* column) {

		std::stringstream buffer(column);

		for (int i = 0; i < rows; ++i)
			buffer >> at(i, j);

	}

	void Matrix::set_values(char const* values) {

		std::stringstream buffer(values);

		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < columns; ++j)
				buffer >> at(i, j);

	}

	// Matrix::storage_accessor
	long double& Matrix::at(int r, int c) {
		if (r >= rows || c >= columns)
			throw(IndexOutOfBoundException());

		//long double& dest = values->at(r * columns + c);
		//return Matrix::storage_accessor(dest, *this);
		return values->at(r * columns + c);
	}

	long double const& Matrix::at(int r, int c) const {
		if (r >= rows || c >= columns)
			throw(IndexOutOfBoundException());

		//long double& dest = values->at(r * columns + c);
		return values->at(r * columns + c);
	}

	AnonymousMatrix Matrix::operator*(Matrix const& m) {
		if (columns != m.rows)
			throw(SizeMismatchException());

		// Allocate return matrix filled with zeroes
		AnonymousMatrix r(rows, m.columns);

		// Core computation
		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < m.columns; ++j)
				for (int h = 0; h < columns; ++h)
					r(i, j) += at(i, h) * m(h, j);
		return r;
	}

	Matrix& Matrix::operator*=(Matrix const& m) {

		// Multiplication can be carried out
		if (columns != m.rows)
			throw(SizeMismatchException());

		// No fear to change matrix size
		Matrix r(rows, m.columns);

		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < m.columns; ++j)
				for (int h = 0; h < columns; ++h)
					r(i, j) += at(i, h) * m(h, j);

		// Swap contents
		swap(values->contents, r.values->contents);
		rows = r.rows;
		columns = r.columns;

		return *this;
	}


	Matrix& Matrix::operator*=(AnonymousMatrix m) {

		// Multiplication can be carried out
		if (columns != m.rows)
			throw(SizeMismatchException());

		// No fear to change matrix size
		Matrix r(rows, m.columns);

		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < m.columns; ++j)
				for (int h = 0; h < columns; ++h)
					r(i, j) += at(i, h) * m(h, j);

		// Swap contents
		swap(values->contents, r.values->contents);
		rows = r.rows;
		columns = r.columns;

		return *this;
	}

	AnonymousMatrix Matrix::operator*(AnonymousMatrix m) const {

		if (columns != m.rows)
			throw(SizeMismatchException());

		// Allocate return matrix filled with zeroes
		AnonymousMatrix r(rows, m.columns);

		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < m.columns; ++j)
				for (int h = 0; h < columns; ++h)
					r(i, j) += at(i, h) * m(h, j);

		// Swap pointers
		swap(m.values->contents, r.values->contents);                         // Keep m's counter
		m.rows = r.rows;
		m.columns = r.columns;

		return m;
	}


	Matrix& Matrix::operator=(char const * values) {

		if (rows == 0 && columns == 0) {
			int chunks = 0;
			long double ignore;
			std::stringstream buffer(values);
			while (!buffer.eof()) {
				buffer >> ignore;
				++chunks;
			}
			resize(1, chunks);
		}
		set_values(values);
		return *this;
	}

	Matrix& Matrix::operator=(Matrix const& m) {

		// Handle autoassignment
		if (&m == this)
			return *this;

		// Scalars
		rows = m.rows;
		columns = m.columns;

		// Storage
		delete values;                                                          // Free old values
		values = new storage(*m.values);                                        // Deep copy of values

		return *this;
	}

	Matrix& Matrix::operator=(AnonymousMatrix m) {

		// Swap values, m will destroy old Matrix values
		swap(values->contents, m.values->contents);                          // Does not swap counters

		// Handle scalar values
		rows = m.rows;
		columns = m.columns;

		return *this;
	}

	AnonymousMatrix Matrix::operator-(AnonymousMatrix  m) const {

		if (dim() != m.dim())
			throw(SizeMismatchException());

		AnonymousMatrix r(rows, columns);

		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < columns; ++j)
				r(i, j) = at(i, j) - m(i, j);

		return r;
	}

	AnonymousMatrix Matrix::operator+(AnonymousMatrix  m) const {

		if (dim() != m.dim())
			throw(SizeMismatchException());

		AnonymousMatrix r(rows, columns);

		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < columns; ++j)
				r(i, j) = at(i, j) + m(i, j);

		return r;
	}

	// Utility functions
	pair<int, int> Matrix::dim() const {
		return make_pair(rows, columns);
	}

	double Matrix::space() const {
		return (rows * columns * sizeof(long double) * 0.000000954);
	}

	// Aux
	bool Matrix::more_equal_than(long double value, long double tol = 0.0000000000000001) const {
		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < columns; ++j)
				if (at(i, j) + tol < value) return false;
		return true;
	}

	bool Matrix::less_equal_than(long double value, long double tol = 0.0000000000000001) const {

		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < columns; ++j)
				if (at(i, j) - tol > value) return false;
		return true;
	}

	Matrix::operator long double() {
		if (dim() != make_pair(1, 1))
			throw (SizeMismatchException());
		return values->at(0);
	}

	void Matrix::log(std::string name) const {
		// Printing
		printf("-- %s\n", name.c_str());
		for (int i = 0; i < rows; ++i) {
			printf(" ");
			for (int j = 0; j < columns; ++j) {
				printf("%10.5f ", (double)at(i, j));
			}
			printf("\n");
		}
		printf("--\n");
	}

	void Matrix::logtave(std::string varname) const {
		// Printing
		printf("%s = ...\n[", varname.c_str());
		for (int i = 0; i < rows; ++i) {

			for (int j = 0; j < columns; ++j) {

				printf("%.16f ", (double)at(i, j));
			}

			printf(";\n");
		}
		printf("]\n");
	}

	bool Matrix::is_square() const {
		return (rows == columns);
	}

	void Matrix::swap_columns(int r, int w) {

		for (int i = 0; i < rows; ++i)
			swap(values->contents->at(i * columns + r), values->contents->at(i * columns + w));

		// Waiting for a smarter implementation
		determinant_up_to_date = false;
		lu_up_to_date = false;
		inverse_up_to_date = false;
	}

	void Matrix::swap_rows(int r, int w) {

		for (int i = 0; i < columns; ++i)
			swap(values->contents->at(r * columns + i), values->contents->at(w * columns + i));

		// Waiting for a smarter implementation
		determinant_up_to_date = false;
		lu_up_to_date = false;
		inverse_up_to_date = false;
	}


	bool Matrix::is_identity(long double tol) const {

		// Identity check
		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < columns; ++j)
				if (((i == j) && !tol_equal(at(i, j), 1, tol)) ||
					((i != j) && !tol_equal(at(i, j), 0, tol)))
					return false;
		return true;
	}

	void Matrix::set_identity() {

		if (!is_square())
			throw(MatrixNotSquareException());

		// Set identity
		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < columns; ++j)
				if (i == j)
					at(i, j) = 1;
				else
					at(i, j) = 0;

		det = 1;
		determinant_up_to_date = true;
	}

	void Matrix::set_determinant(long double d) {

		det = d;
		determinant_up_to_date = true;
	}

	long double Matrix::determinant() const {

		// Return determinant if cached, else factorize and return it
		if (!determinant_up_to_date) {
			Matrix l, u, p;
			get_lupp(l, u, p, PF_VECTOR);
		}
		return det;
	}

	void Matrix::transpose() {

		// If matrix is square just swap the elements
		if (rows == columns) {
			for (int j = 1; j < columns; ++j)
				for (int i = 0; i < j; ++i)
					swap(at(i, j), at(j, i));
			return;
		}

		AnonymousMatrix tps(columns, rows);
		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < columns; ++j)
				tps(j, i) = at(i, j);

		*this = tps;
	}

	void Matrix::empty() {
		delete values;
		values = new storage(rows * columns);
	}

	void Matrix::resize(int r, int c) {

		// Reinitialize values
		delete values;
		values = new storage(r * c);

		// Resize
		rows = r;
		columns = c;

		// Invalid determinant
		determinant_up_to_date = false;
	}

	AnonymousMatrix Matrix::gaussian_elimination() {

		AnonymousMatrix r(*this);

		int j = 0, pivot = 0;
		while (j < columns && pivot < rows) {

			// Reset tem and tpm elements
			Matrix tem(rows, 1, 0);
			tem(pivot, 0) = 1;

			int column_max_position = pivot;
			long double max = r(column_max_position, j);

			// Partial pivoting process
			for (int i = j; i < rows; ++i)
				if (fabs(r(i, j)) > fabs(max)) {
					column_max_position = i;
					max = r(i, j);
				}

			if (max != 0) {

				// Update U and P with TPM only if necessary
				if (j != column_max_position)
					r.swap_rows(pivot, column_max_position);

				// Write tem vector for current column
				for (int i = pivot + 1; i < rows; ++i)
					tem(i, 0) = -(r(i, j) / max);

				// Optimization of tem * u that takes into account the
				// shape of tem and u
				for (int i = pivot + 1; i < rows; ++i) {
					Matrix t(1, columns, 0);

					for (int o = pivot; o < columns; ++o)
						t(o) = tem(i, 0) * r(j, o) + r(i, o);


					for (int k = pivot; k < columns; ++k)
						r(i, k) = t(k);

				}
				++pivot;
			}
			++j;
		}

		return r;

	}

	bool Matrix::columns_linearly_independent() {

		if (columns > rows)
			return false;

		// Rows -> Columns    
		transpose();

		Matrix test = gaussian_elimination();

		// Back Columns -> Rows
		transpose();

		bool a_row_is_zero = false;

		for (int i = 0; i < test.rows; ++i) {
			bool row_is_zero = true;
			for (int j = 0; j < test.columns; j++)
				if (test(i, j) != 0) {
					row_is_zero = false;
					break;
				}
			if (row_is_zero) {
				a_row_is_zero = true;
				break;
			}
		}

		return !a_row_is_zero;

	}

	bool Matrix::rows_linearly_independent() {

		if (rows > columns)
			return false;

		Matrix test = gaussian_elimination();

		bool a_row_is_zero = false;

		for (int i = 0; i < test.rows; ++i) {
			bool row_is_zero = true;
			for (int j = 0; j < test.columns; j++)
				if (test(i, j) != 0) {
					row_is_zero = false;
					break;
				}
			if (row_is_zero) {
				a_row_is_zero = true;
				break;
			}
		}

		return !a_row_is_zero;
	}

	// LU factorization with Gaussian Elimination and Partial Pivoting
	void Matrix::get_lupp(Matrix& l, Matrix& u, Matrix& p, PermutationFormat pf = PF_MATRIX) const {

		if (!is_square())
			throw(MatrixNotSquareException());

		// Initialize determinant
		long double determinant = 1;

		// Initialize passed u
		u = *this;					                                            // u will evolve from the original matrix

		// Resize passed l
		l.resize(rows, columns);                                                // total l
		l.set_identity();

		// Initialize permutation matrix (if needed)
		if (pf == PF_MATRIX) {                                                  // total p
			p.resize(rows, columns);
			p.set_identity();
		}

		// p_vector and o_vector for efficient permutation handling
		AnonymousMatrix p_vector(rows, 1);
		AnonymousMatrix o_vector(rows, 1);

		// Initialize p_vector, o_vector
		for (int i = 0; i < rows; ++i) {
			p_vector(i) = i;
			o_vector(i) = i;
		}


		for (int j = 0; j < columns; ++j) {

			// Reset tem and tpm elements
			Matrix tem(rows, 1, 0);
			tem(j) = 1;

			// Write tpm:
			//   *  find absolute maximum element j in column i
			//   *  swap row i with row j in p and u, swap columns in l

			int column_max_position = j;
			long double max = u(column_max_position, j);

			// Partial pivoting process
			for (int i = j; i < rows; ++i)
				if (fabs(u(i, j)) > fabs(max)) {
					column_max_position = i;
					max = u(i, j);
				}

			// If matrix is not singular proceed ..
			if (max == 0)
				throw (MatrixIsSingularException());

			// Update U and P with TPM only if necessary
			if (j != column_max_position) {

				// Update determinant sign
				determinant = -determinant;

				// Effects of permutation on l and u
				u.swap_rows(j, column_max_position);
				l.swap_columns(j, column_max_position);

				// Effects on permutation on p and p_vector
				if (pf == PF_MATRIX)
					p.swap_rows(j, column_max_position);
				p_vector.swap_rows(j, column_max_position);

				// If we're returning the PF_MATRIX set its determinant
				if (pf == PF_MATRIX)
					p.set_determinant(determinant);
			}

			// Write tem vector for current column
			for (int i = j + 1; i < rows; ++i)
				tem(i) = -(u(i, j) / max);

			// Optimization of l * tem that takes into account the shape
			// of l and tem
			for (int i = 0; i < rows; ++i) {
				register long double inv_product = l(i, j);   // because tem(j,0) == 1

				for (int k = j + 1; k < columns; ++k)
					inv_product += l(i, k) * -tem(k);

				l(i, j) = inv_product;
			}

			// Optimization of tem * u that takes into account the
			// shape of tem and u
			for (int i = j + 1; i < rows; ++i) {
				Matrix r(1, columns, 0);

				for (int o = j; o < columns; ++o)
					r(o) = tem(i) * u(j, o) + u(i, o);

				for (int k = j; k < columns; ++k)
					u(i, k) = r(k);
			}
		}

		// Optimized way to calculate p * l, a permutation vector
		// is used to swap the rows of l        
		for (int i = 0; i < rows; ++i)
			while (p_vector(i) != o_vector(i)) {
				int k = i + 1;
				while (p_vector(k) != o_vector(i))
					k++;

				o_vector.swap_rows(i, k);
				l.swap_rows(i, k);
			}


		// Return PF_VECTOR in p
		if (pf == PF_VECTOR)
			p = p_vector;

		// Compute and set determinant        
		for (int i = 0; i < rows; ++i)
			determinant *= u(i, i);

		determinant_up_to_date = true;

	}

	void Matrix::get_inverse(Matrix& inverse) const {

		if (!is_square())
			throw(MatrixNotSquareException());

		// Adjust inverse size
		inverse.resize(rows, columns);

		// Temporary matrices to hold factorization products
		Matrix l_inverse, u_inverse, p_vector;

		// Compute and store LUPP
		get_lupp(l_inverse, u_inverse, p_vector, PF_VECTOR);

		// Set original permutation vector
		Matrix o_vector(rows, 1);
		for (int i = 0; i < rows; ++i)
			o_vector(i) = i;

		// Copy transposed l
		l_inverse.transpose();

		// Set reciprocals on the diagonal of u, useless in l since they are ones
		for (int i = 0; i < rows; ++i)
			u_inverse(i, i) = 1 / u_inverse(i, i);

		// Calculate inverse of l
		for (int i = 1; i < rows; ++i)
			for (int j = i - 1; j >= 0; --j) {
				register long double dot_product = 0;
				for (int k = i; k > 0; --k)
					dot_product += l_inverse(i, k) * l_inverse(j, k);
				l_inverse(i, j) = -dot_product;                                 // Optimization of dot_product * - l_inverse.at(j,j)
			}

		// Set zeroes on the upper half of l^-1
		for (int i = 0; i < rows; ++i)
			for (int j = i + 1; j < columns; ++j)
				l_inverse(i, j) = 0;

		// Calculate inverse of u
		for (int i = 1; i < rows; ++i)
			for (int j = i - 1; j >= 0; --j) {
				register long double dot_product = 0;
				for (int k = i; k > 0; --k) {
					dot_product += u_inverse(i, k) * u_inverse(j, k);
				}
				u_inverse(i, j) = dot_product * -u_inverse(j, j);
			}

		u_inverse.transpose();


		// Set zeroes on the lower half of u^-1
		for (int j = 0; j < columns; ++j)
			for (int i = j + 1; i < rows; ++i)
				u_inverse(i, j) = 0;



		// Optimization of u^-1 * l^-1 that takes into account the
		// shape of the two matrices
		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < columns; ++j)
				for (int h = columns - 1; h >= std::min(i, j); --h)
					inverse(i, j) += u_inverse(i, h) * l_inverse(h, j);

		// Smart way to translate a row permutation vector to obtain
		// a column permutation vector
		Matrix p_vector_t(rows, 1);
		for (int i = 0; i < rows; ++i)
			p_vector_t(p_vector(i)) = i;

		// Optimization of columns permutation
		for (int i = 0; i < rows; ++i)
			while (p_vector_t(i) != o_vector(i)) {
				int k = i + 1;
				while (p_vector_t(k) != o_vector(i))
					k++;

				o_vector.swap_rows(i, k);
				inverse.swap_columns(i, k);
			}
	}

	void Matrix::get_inverse_with_column(Matrix const& old_inverse,
		Matrix const& new_column,
		int q,
		Matrix& new_inverse) {

		// Prepare result
		new_inverse.resize(old_inverse.rows, old_inverse.columns);
		Matrix a_tilde(old_inverse * new_column);

		for (int i = 0; i < old_inverse.rows; ++i)
			for (int j = 0; j < old_inverse.columns; ++j)
				if (i != q)
					new_inverse(i, j) = old_inverse(i, j) - ((old_inverse(q, j) * a_tilde(i)) / a_tilde(q));
				else
					new_inverse(i, j) = old_inverse(q, j) / a_tilde(q);


	}


	void Matrix::solve(Matrix& x, Matrix const& b) const {

		// Invert L and U
		Matrix l_inverse, u_inverse, p_vector;

		// Calculate LUPP factorization
		get_lupp(l_inverse, u_inverse, p_vector, PF_VECTOR);

		// Copy transposed l
		l_inverse.transpose();

		// Set reciprocals on the diagonal of u (useless in l since diagonal elements are ones)
		for (int i = 0; i < rows; ++i)
			u_inverse(i, i) = 1 / u_inverse(i, i);

		// Calculate inverse of l
		for (int i = 1; i < rows; ++i)
			for (int j = i - 1; j >= 0; --j) {
				register long double dot_product = 0;
				for (int k = i; k > 0; --k)
					dot_product += l_inverse(i, k) * l_inverse(j, k);
				l_inverse(i, j) = -dot_product;                                 // Optimization due to ones on diagonal
			}

		// Set zeroes on the upper half of l^-1
		for (int i = 0; i < rows; ++i)
			for (int j = i + 1; j < columns; ++j)
				l_inverse(i, j) = 0;

		// Calculate inverse of u
		for (int i = 1; i < rows; ++i)
			for (int j = i - 1; j >= 0; --j) {
				register long double dot_product = 0;
				for (int k = i; k > 0; --k) {
					dot_product += u_inverse(i, k) * u_inverse(j, k);
				}
				u_inverse(i, j) = dot_product * -u_inverse(j, j);
			}

		u_inverse.transpose();

		// Set zeroes on the lower half of u^-1
		for (int j = 0; j < columns; ++j)
			for (int i = j + 1; i < rows; ++i)
				u_inverse(i, j) = 0;

		// Optimization of p * b
		Matrix pb(rows, 1);
		for (int i = 0; i < rows; ++i)
			pb(i) = b(p_vector(i));

		// Set x shape	
		x.resize(rows, 1);

		// Optimization of x = l_inverse * pb;
		for (int i = 0; i < rows; ++i) {
			register long double dot_product = pb(i);
			for (int j = 0; j < i; ++j) {
				dot_product += l_inverse(i, j) * pb(j);
			}
			x(i) = dot_product;
		}

		// Optimization of x = u_inverse * x
		for (int i = 0; i < rows; ++i) {
			register long double dot_product = 0;
			for (int j = columns - 1; j >= i; --j)
				dot_product += u_inverse(i, j) * x(j);
			x(i) = dot_product;
		}
	}

	/*
	Matrix::storage_accessor
	*/
	Matrix::storage_accessor::storage_accessor(long double& dest, Matrix& parent) :
		dest(dest),
		parent(parent) {
	}

	Matrix::storage_accessor::operator long double const& () const {
		return dest;
	}

	Matrix::storage_accessor& Matrix::storage_accessor::operator=(Matrix::storage_accessor& new_value) {

		if (new_value.dest == dest)
			return *this;

		dest = new_value.dest;

		// Update bools
		parent.determinant_up_to_date = false;
		parent.inverse_up_to_date = false;
		parent.lu_up_to_date = false;

		return *this;
	}

	Matrix::storage_accessor& Matrix::storage_accessor::operator=(long double const& new_value) {

		if (new_value == dest)
			return *this;

		dest = new_value;

		// Update bools
		parent.determinant_up_to_date = false;
		parent.inverse_up_to_date = false;
		parent.lu_up_to_date = false;

		return *this;
	}

	/*
	Matrix::storage
	===============
	Reference counted vector of long doubles.

	*/

	Matrix::storage::storage(int size) :
		contents(new std::vector< long double>(size)),
		counter(1) {
	}

	Matrix::storage::storage(Matrix::storage& s) :
		contents(new std::vector< long double>(*(s.contents))),
		counter(1) {
	}

	Matrix::storage::storage(int size, long double value) :
		contents(new std::vector< long double>(size, value)),
		counter(1) {
	}

	long double& Matrix::storage::at(int pos) {
		return contents->at(pos);
	}

	Matrix::storage::~storage() {
		// Decrease reference count
		--counter;

		// Reference count goes to zero
		if (counter == 0) {
			delete contents;
		}
	}


	/*
	AnonymousMatrix
	===============
	Matrix that is generate from scratch during the calculations.

	*/

	AnonymousMatrix AnonymousMatrix::operator*(Matrix const& m) {

		if (columns != m.rows)
			throw(SizeMismatchException());

		// Allocate return matrix filled with zeroes
		AnonymousMatrix r(rows, m.columns);

		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < m.columns; ++j)
				for (int h = 0; h < columns; ++h)
					r(i, j) += at(i, h) * m(h, j);

		// Swap pointers
		rows = r.rows;
		columns = r.columns;
		swap(values->contents, r.values->contents);

		return *this;
	}

	AnonymousMatrix::AnonymousMatrix(const AnonymousMatrix& m) : Matrix(m.rows, m.columns) {

		swap(Matrix::values->contents, m.values->contents);

	}

	// Other constructors	
	AnonymousMatrix::AnonymousMatrix(const Matrix& m) : Matrix(m) {	}
	AnonymousMatrix::AnonymousMatrix(int r, int c) : Matrix(r, c) { }

	// Auxiliary 

	bool tol_equal(long double n, long double m, long double tol = 0.0000000000000001) {
		if (abs(n - m) > tol)
			return false;
		return true;
	}



}