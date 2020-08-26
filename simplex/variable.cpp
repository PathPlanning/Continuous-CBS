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

#include "variable.h"

namespace optimization {

	/*
	Variable
	========
	Base variable class.

	*/

	Variable::Variable(Simplex* creator, char const * name) :
		name(name),
		creator(creator) {
	}

	Variable::~Variable() { }

	void Variable::process(Matrix& calculated_solution, Matrix& solution, int index) {
		solution(index) = calculated_solution(index);
	}

	/*
	SplittedVariable
	================
	Variables that are splitted in two (one) AuxiliaryVariables during
	the translation in standard form.

	*/

	SplittedVariable::SplittedVariable(Simplex* creator, char const * name, AuxiliaryVariable* aux) :
		Variable(creator, name),
		aux(aux) {
	}

	SplittedVariable::~SplittedVariable() { }

	void SplittedVariable::process(Matrix& calculated_solution, Matrix& solution, int index) {
		solution(index) = calculated_solution(index) - calculated_solution(aux->index);
	}

	/*
	SlackVariable
	=============
	Type of variable added when transforming a <= or >= constraint
	into a = constraint.

	*/

	SlackVariable::SlackVariable(Simplex* creator, char const * name) :
		Variable(creator, name) {
	}

	SlackVariable::~SlackVariable() { }

	void SlackVariable::process(Matrix& calculated_solution, Matrix& solution, int index) {}

	/*
	AuxiliaryVariable
	=================
	Variable created when transforming a variable in a splitted
	variable. The relation:

	x = x+ - x-

	holds between the original variable, the SplittedVariable and
	the AuxiliaryVariable.

	*/

	AuxiliaryVariable::AuxiliaryVariable(Simplex* creator, char const * name, int index) :
		Variable(creator, name),
		index(index) {

	}

	AuxiliaryVariable::~AuxiliaryVariable() { }

	void AuxiliaryVariable::process(Matrix& calculated_solution, Matrix& solution, int index) {}
}