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


// This header defines some numerical constants and math functions.

#ifndef __LOVIMATH_H
#define __LOVIMATH_H

#include <cmath>
#include <limits>

using namespace std;

namespace dlovi {
	// General mathematical constants
	const double pi = 3.141592653589793238462643;
	const double ln2 = 0.69314718055994530942;
	const double e = 2.718281828459045235360287;

	// Constants for golden section searches, etc
	const double golden_ratio = 1.6180339887498948482;
	const double golden_ratio_conjugate = 0.618033988749895;
	const double golden_section = 0.381966011250105;

	// Machine precision related constants
	const double eps_d = pow(2.0, -52.0);
	const double eps_f = pow(2.0, -23.0);
	const double sqrt_eps_d = 1.49011611938477e-08;
	const double sqrt_eps_f = 3.45266983001244e-04;
	const double realmin_d = ldexp(1.0, -1022);
	const double realmin_f = (float)ldexp(1.0, -126);

	// Constants related to Inf and NaN
	const double NaN = numeric_limits<double>::quiet_NaN();
	const double Inf = numeric_limits<double>::infinity();
	const double inf = numeric_limits<double>::infinity();

	// (Most) Functions Declarations
	int floatEquals(float, float);
	int doubleEquals(double, double);

	double eps(double x);
	float eps(float x);

	int round(double x);
	int round(float x);

	bool isNaN(double x);

	// Special functions
	double sinc(double x);

	// Templated functions:

	template <class T> 
  T sign(const T x) { // computes the signum
    if (x > T(0))
			return T(1);
    else if (x < T(0))
			return T(-1);
		else
			return T(0);
	}

	template <class T>
  T replaceSign(const T a, const T b) { // returns a value with the same magnitude as a and the same sign as b.  (if sign(b) == 0, sign of a unchanged)
		T nSignb = sign(b);
		T nSigna = sign(a); 
		return a * nSigna * ( (nSignb == T(0)) ? nSigna : nSignb );
	}
}
#endif

