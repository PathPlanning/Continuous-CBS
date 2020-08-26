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

#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "pilal.h"
using pilal::Matrix;

namespace optimization {

	enum ConstraintType {

		CT_LESS_EQUAL,
		CT_MORE_EQUAL,
		CT_EQUAL,
		CT_NON_NEGATIVE,
		CT_BOUNDS

	};


	class Constraint {

		friend class Simplex;

	public:

		Constraint(Matrix const & coefficients, ConstraintType type, long double value);
		Constraint(Matrix const & coefficients, ConstraintType type, long double lower, long double upper);

		// Debug
		void log() const;
		void add_column(long double value);
		int size() const;

	private:

		ConstraintType type;
		Matrix coefficients;
		long double value;
		long double upper;
		long double lower;

	};

}

#endif