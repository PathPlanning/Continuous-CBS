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

#ifndef MATRIX_H
#define MATRIX_H

#include "pilal.h"
#include <utility>	// std::pair
#include <vector>	// std::vector
#include <iostream> // std::string

namespace pilal {

	/** Enum type that describes the two types of data used to perform
	permutations: permutation matrices or permutation vectors (explained
	later).  */
	enum PermutationFormat {
		PF_MATRIX,
		PF_VECTOR
	};

	/** Enum type that describes the shape of the matrix to optimize
	matrix inversion for special matrices (triangular, permutation
	or generic). */

	enum MatrixType {
		MT_GENERIC,
		MT_TRIANGULAR_UPPER,
		MT_TRIANGULAR_LOWER,
		MT_PERMUTATION
	};

	/** Forward declaration for matrix and anonymous Matrix. */
	class Matrix;
	class AnonymousMatrix;

	/** Represents a matrix. */
	class Matrix {

		/** Provide access to anonymous matrices. */
		friend class AnonymousMatrix;

	public:

		/** Default constructor. */
		Matrix();

		/** Constructor which accepts a string of values and builds a single row matrix. */
		Matrix(char const * values);

		/** Constructur which builds a square matrix of dimension n. */
		Matrix(int n);

		/** Constructor which builds a square matrix of dimension n, and initializes each element to v. */
		Matrix(int n, long double v);

		/** Constructor which builds a r x c matrix. */
		Matrix(int r, int c);

		/** Constructor which builds a r x c matrix, and initializes each element to v. */
		Matrix(int r, int c, long double v);

		/** Copy constructor. */
		Matrix(Matrix const& m);

		/** Create a matrix from an anonymous matrix (copies data pointer). */
		Matrix(AnonymousMatrix m);
		virtual ~Matrix();

		/** Accessor for data elements, can be modified to support caching. */
		class storage_accessor {

		public:

			/** Constructor, accepts a reference to a value and a parent matrix. */
			storage_accessor(long double& dest, Matrix& parent);

			/** Implicit cast operator, used in reading. */
			operator long double const& () const;                           // Reading


			storage_accessor& operator=(storage_accessor& new_value);       // Copying
			storage_accessor& operator=(long double const& new_value);      // Writing

		private:

			/** Reference to real value. */
			long double& dest;

			/** Owner. */
			Matrix& parent;
		};

		/*=========================================================
		Query and log operators
		=========================================================*/

		/** Dimension of the matrix. */
		virtual std::pair<int, int> dim() const;

		/** Prints the matrix with a name for debug. */
		void log(std::string name) const;

		/** Prints the matrix in a format compatible with octave. */
		void logtave(std::string name) const;

		/** Is the matrix square? */
		bool is_square() const;

		/** Is the matrix an identity (with tolerance value)? */
		bool is_identity(long double tol) const;

		/** How much storage space does the matrix uses? */
		double space() const;

		/** Compare two values (with tolerance). */
		bool more_equal_than(long double value, long double tol) const;

		/** Compare two values (with tolerance). */
		bool less_equal_than(long double value, long double tol) const;

		/*=========================================================
		Mathematical and manipulation operators
		=========================================================*/

		/** Subtracts a matrix and an anonymous matrix, generates an anonymous matrix. */
		AnonymousMatrix operator- (AnonymousMatrix m) const;

		/** Adds a matrix to an anonymous matrix, generates an anonymous matrix. */
		AnonymousMatrix operator+ (AnonymousMatrix m) const;

		/** Swaps columns r and w in the matrix. */
		void swap_columns(int r, int w);

		/** Swaps rows r and w in the matrix. */
		void swap_rows(int r, int w);

		/** Writes the value of the determinant. */
		void set_determinant(long double d);

		/** Reset the matrix to an identity. */
		void set_identity();

		/** Transposes the matrix. */
		void transpose();

		/** Set the matrix size t r x c.*/
		void resize(int r, int c);

		/** Fills the matrix with zeroes. */
		void empty();

		/** Fills a row of the matrix with elements in string row. */
		void set_row(int i, char const* row);

		/** Fills a column of the matrix with elements in string column. */
		void set_column(int j, char const* column);

		/** Fills the matrix with the elements in values. */
		void set_values(char const* values);

		/** Retrieves the determinant of the matrix.  */
		long double determinant() const;

		/*=========================================================
		Factorizations and inverses
		=========================================================*/

		/** Performs a LU factorization and stores the l, u matrices and permutation data respective
		into l, u and p. The third parameter determines the format of permutation data p (vector or matrix). */
		void get_lupp(Matrix& l, Matrix& u, Matrix& p, PermutationFormat pf) const;

		/** Get inverse of the matrix and stores it into inverse. */
		void get_inverse(Matrix& inverse) const;

		/** Get inverse of the matrix and stores it into a matrix of type mt. */
		void get_inverse(Matrix& inverse, MatrixType mt) const;

		/** Updates inverse of the matrix after a column has changed. */
		static void get_inverse_with_column(Matrix const& old_inverse, Matrix const& new_column, int column_index, Matrix& new_inverse);

		/** Solves the linear problem represented by the matrix using the data vector b. */
		void solve(Matrix& x, Matrix const& b) const;

		/** Checks if rows are linearly independent. */
		bool rows_linearly_independent();

		/** Checks if columns are linearly independent. */
		bool columns_linearly_independent();

		/*=========================================================
		Multiplication operators
		=========================================================*/

		/** Matrix multiplication operator with anonymous matrix. */
		virtual AnonymousMatrix operator*(AnonymousMatrix m) const;

		/** Matrix multiplication operator. */
		virtual AnonymousMatrix operator*(Matrix const& m);

		/** Assignment operator with matrix multiplication. */
		Matrix& operator*=(Matrix const& m);

		/** Assignment operator with anonymous matrix multiplication. */
		Matrix& operator*=(AnonymousMatrix m);

		/*=========================================================
		Assignments
		=========================================================*/

		/** Assignment operator with matrix. */
		Matrix& operator=(Matrix const& m);

		/** Assignment operator with values string. */
		Matrix& operator=(char const * values);

		/** Assignment operator with anonymous matrix. */
		Matrix& operator=(AnonymousMatrix m);

		/*=========================================================
		Retrieval and cast operators
		=========================================================*/

		/** Element retrieval with one index. */
		long double& operator() (int i);

		/** Element retrieval with one index (const). */
		long double const& operator() (int i) const;

		/** Element retrieval with two idices. */
		long double& operator() (int r, int c);

		/** Element retrieval with two indices (const). */
		long double const& operator() (int r, int c) const;

		/** Element retrieval with one index. */
		long double& at(int r, int c);

		/** Element retrieval with one index (const). */
		long double const& at(int r, int c) const;

		/** Implicit cast to double. */
		operator long double();

	protected:

		/** Procedure for Gaussian elimination. */
		AnonymousMatrix gaussian_elimination();

		/** Retrieves matrix type (anonymous or regular). */
		MatrixType get_matrix_type(Matrix const& m) const;

		/** Cache information. */
		mutable bool lu_up_to_date, determinant_up_to_date, inverse_up_to_date;

		/** Storage class, a reference counted pointer to heap-allocated data. */
		class storage {

		public:

			/** Constructor, destructor. */
			storage(int size);
			storage(int size, long double value);
			storage(storage& origin);
			~storage();

			/** Access operator. */
			long double & at(int pos);

			/** Pointer to data. */
			std::vector< long double> * contents;

			/** Reference count. */
			int counter;

		};

		/** Pointer to heap-allocated storage (for implementing anonymous matrices in an efficient way). */
		storage* values;

		/** Number of rows. */
		int rows;

		/** Number of columns. */
		int columns;

		/** Determinant. */
		mutable long double det;

	};

	/** Represents a matrix for temporary use. */
	class AnonymousMatrix : public Matrix {

	public:

		/** Constructor and copy constructors. */
		AnonymousMatrix(int r, int c);
		AnonymousMatrix(const AnonymousMatrix& m);
		AnonymousMatrix(const Matrix& m);

		/** Multiplication operator. */
		AnonymousMatrix operator*(Matrix const& m);
	};

	/** Auxiliary function, number comparison with tolerance. */
	bool tol_equal(long double n, long double m, long double tol);
}

#endif