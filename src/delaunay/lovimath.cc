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

#ifndef __LOVIMATH_CPP
#define __LOVIMATH_CPP

#include "lovimath.h"

namespace dlovi {

  int floatEquals(float x, float y) {
		return fabs(x - y) < eps_f;
	}

  int doubleEquals(double x, double y) {
		return fabs(x - y) < eps_d;
	}

  double eps(double x) {
		if (fabs(x) <= realmin_d)
			return pow(2.0, -1074.0);
    else {
			//2^E <= abs(X) < 2^(E+1)  -->  E <= log_2(abs(X)) < E+1....
			//true iff E = floor(log_2(abs(X))).  Floor can be done by casting 	//to int.
			//so......
			//int E = (int)log_2(fabs(x));

			return pow(2.0, (double)((int)(log(fabs(x)/ln2)) - 52));
		}
	}

  float eps(float x) {
		if (fabs(x) <= realmin_f)
			return pow(2.0, -149.0);
    else {
			//2^E <= abs(X) < 2^(E+1)  -->  E <= log_2(abs(X)) < E+1....
			//true iff E = floor(log_2(abs(X))).  Floor can be done by casting 	//to int.
			//so......
			//int E = (int)log_2(fabs(x));

			return (float)pow(2.0, (double)((int)(log(fabs(x)/ln2)) - 23));
		}
	}

  int round(double x) {
    if (x >= 0.0)
			return (int)(x + 0.5);
		return (int)(x - 0.5);
	}

  int round(float x) {
    if (x >= 0.0f)
			return (int)(x + 0.5f);
		return (int)(x - 0.5f);
	}

  bool isNaN(double x) {
		return x != x;
	}

  double sinc(double x) {
    if (x == 0.0)
			return 1;
		return sin(x) / x;
	}
}

#endif
