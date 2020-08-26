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

#ifndef OBJECTIVE_FUNCTION_H
#define OBJECTIVE_FUNCTION_H

#include "pilal.h"
using pilal::Matrix;

namespace optimization {

	enum ObjectiveFunctionType {

		OFT_MAXIMIZE,
		OFT_MINIMIZE

	};

	class ObjectiveFunction {

		friend class Simplex;

	public:

		ObjectiveFunction();
		ObjectiveFunction(ObjectiveFunctionType type, Matrix const & costs);
		ObjectiveFunction& operator=(ObjectiveFunction const & objective_function);

		// Solution value
		Matrix const & get_value(Matrix const & x) const;

		// Manipulation
		void add_column(long double value);

		// Debug
		void log() const;

	private:

		ObjectiveFunctionType type;
		Matrix costs;

	};

}

#endif