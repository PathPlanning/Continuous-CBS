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

#include "constraint.h"
#include "simplex.h"

// Using
using pilal::Matrix;
using pilal::AnonymousMatrix;

namespace optimization {

	/*
	Constraint
	==========
	Class that represents a constraint: A_i * x_j = b_i.

	*/

	Constraint::Constraint(Matrix const & coefficients, ConstraintType type, long double value) {

		// Coefficients must be a row vector
		if (coefficients.dim().first == 1) {
			this->coefficients = coefficients;
			this->type = type;
			this->value = value;
		}
		else {
			throw(DataMismatchException("Invalid coefficients vector."));
		}
	}

	Constraint::Constraint(Matrix const & coefficients, ConstraintType type, long double lower, long double upper) {

		if (type != CT_BOUNDS)
			throw(DataMismatchException("Invalid constraint type for provided data"));

		// Coefficients must be a row vector
		if (coefficients.dim().first == 1) {
			this->coefficients = coefficients;
			this->type = type;
			this->lower = lower;
			this->upper = upper;
		}
		else {
			throw(DataMismatchException("Invalid coefficients vector."));
		}
	}

	int Constraint::size() const {
		return coefficients.dim().second;
	}

	void Constraint::log() const {
		for (int i = 0; i < coefficients.dim().second; ++i)
			std::cout << coefficients(i) << "\t";

		switch (type) {

		case CT_EQUAL:
			std::cout << "=\t";
			break;

		case CT_LESS_EQUAL:
			std::cout << "<=\t";
			break;

		case CT_MORE_EQUAL:
			std::cout << ">=\t";
			break;

		case CT_BOUNDS:
			std::cout << "bounded to ";
			break;

		case CT_NON_NEGATIVE:
			std::cout << "non-negative ";
		}

		if (type == CT_NON_NEGATIVE)
			std::cout << std::endl;
		else if (type == CT_BOUNDS)
			std::cout << lower << " <= " << "value" << " <= " << upper << std::endl;
		else
			std::cout << value << std::endl;
	}

	void Constraint::add_column(long double value) {
		AnonymousMatrix row(1, coefficients.dim().second + 1);
		for (int i = 0; i < coefficients.dim().second; ++i)
			row(i) = coefficients(i);

		row(coefficients.dim().second) = value;
		coefficients = row;
	}

}