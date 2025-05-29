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

#ifndef VARIABLE_H
#define VARIABLE_H

#include "constraint.h"          
#include "simplex.h"

namespace optimization {

	class AuxiliaryVariable;

	class Variable {

		friend class Simplex;

	public:
		Variable(Simplex * creator, char const * name);
		virtual ~Variable();
		virtual void process(Matrix& calculated_solution, Matrix& solution, int index);

	protected:
		std::string name;
		Simplex * creator;

	};

	class SplittedVariable : public Variable {

		friend class Simplex;

	public:
		SplittedVariable(Simplex* creator, char const * name, AuxiliaryVariable* aux);
		~SplittedVariable();
		void process(Matrix& calculated_solution, Matrix& solution, int index);
	private:
		AuxiliaryVariable* aux;

	};

	class SlackVariable : public Variable {

		friend class Simplex;

	public:
		SlackVariable(Simplex * creator, char const * name);
		~SlackVariable();
		void process(Matrix& calculated_solution, Matrix& solution, int index);


	};

	class AuxiliaryVariable : public Variable {

		friend class Simplex;
		friend class SplittedVariable;

	public:
		AuxiliaryVariable(Simplex* creator, char const * name, int index);
		~AuxiliaryVariable();
		void process(Matrix& calculated_solution, Matrix& solution, int index);
	private:
		int index;

	};
}

#endif