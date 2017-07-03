//  Copyright 2011 David Lovi
//
//  This file is part of FreespaceDelaunayAlgorithm.
//
//  FreespaceDelaunayAlgorithm is free software: you can redistribute it
//  and/or modify it under the terms of the GNU General Public License as
//  published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version.
//
//  FreespaceDelaunayAlgorithm is distributed in the hope that it will be
//  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with FreespaceDelaunayAlgorithm.  If not, see
//  <http://www.gnu.org/licenses/>.
//
//  As a special exception, you have permission to link this program
//  with the CGAL library and distribute executables, as long as you
//  follow the requirements of the GNU GPL in regard to all of the
//  software in the executable aside from CGAL.


// This header introduces several convenience functions for processing strings.

#ifndef __DLOVI_STRINGFUNCTIONS_H
#define __DLOVI_STRINGFUNCTIONS_H

#include <string>
#include <vector>
#include <cctype>

namespace dlovi {
  namespace stringfunctions {
		std::vector<std::string> split(const std::string & strOriginal, const std::string & strDelimiter);
		std::string join(const std::vector<std::string> & vStrArray, const std::string & strDelimiter);
    std::string trim(const std::string & strOriginal);
	}
}

#endif
