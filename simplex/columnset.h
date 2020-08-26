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

#ifndef COLUMNSET_H
#define COLUMNSET_H

#include <vector>

namespace optimization {

	class ColumnSet {

		friend class Simplex;

	public:

		void insert(int column);
		void remove(int column);
		void substitute(int old_column, int new_column);
		void log(char const * prelude) const;
		bool contains(int n) const;
		int& column(int idx);
		int  index_of(int column);
		unsigned int size() const;


	private:

		// A int vector stores the indices of the columns in the set.
		std::vector<int> columns;
	};

}


#endif