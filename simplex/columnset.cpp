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

#include "columnset.h"
#include "simplex.h"
#include <algorithm>
#include <vector>

using std::vector;

namespace optimization {

	/*
	ColumnSet
	=========
	Class that represents a set of columns.

	*/

	void ColumnSet::insert(int column) {
		columns.push_back(column);
	}

	void ColumnSet::substitute(int old_column, int new_column) {
		if (find(columns.begin(), columns.end(), old_column) != columns.end())
			*(find(columns.begin(), columns.end(), old_column)) = new_column;
	}

	void ColumnSet::remove(int column) {
		if (find(columns.begin(), columns.end(), column) != columns.end())
			columns.erase(find(columns.begin(), columns.end(), column));
	}

	int ColumnSet::index_of(int column) {
		int pos = 0;
		for (vector<int>::iterator it = columns.begin(); it != columns.end(); ++it)
			if (*it != column)
				++pos;
			else
				return pos;
		return -1;
	}

	void ColumnSet::log(char const * prelude) const {


		std::cout << prelude;

		for (vector<int>::const_iterator it = columns.begin();
			it != columns.end();
			++it) {
			std::cout << *it << " ";
		}

		std::cout << std::endl;
	}

	bool ColumnSet::contains(int column) const {
		return find(columns.begin(), columns.end(), column) != columns.end();
	}

	int& ColumnSet::column(int idx) {
		return columns.at(idx);
	}

	unsigned int ColumnSet::size() const {
		return columns.size();
	}
}