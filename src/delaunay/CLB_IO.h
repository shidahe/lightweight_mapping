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
//
//  Note: The file-format that this code reads & writes is not covered under
//  the GPL and is not owned or copyrighted this file's author.


// This header contains routines for reading and writing .clb files.
// .clb files hold camera calibration information, intrinsic and extrinsic.

#ifndef __CLB_IO_H
#define __CLB_IO_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>

#include <sys/types.h>

namespace CLB_IO {

	#include "Matrix.h"

	using dlovi::Matrix;
	using std::ifstream;
	using std::ofstream;
	using std::stringstream;

  void writeTag(ofstream & outfile, const char * tag, int length) {
		outfile.write(tag, 4);
		outfile.write((char *) & length, 4);
	}

  void writeCLB(const char * fname, Matrix K, Matrix E = Matrix(), Matrix d = Matrix()) {
		// fname: .clb filename to write output to
		// K: 3x3 internals
		// E: 4x4 externals (optional)
		// d: 5-vector distortion coefficients (optional)

		// Open the file
		ofstream outfile(fname, ios::out | ios::binary);
    if(! outfile.is_open()) { stringstream ssErr;  ssErr << "Unable to open file: " << fname; throw ssErr.str().c_str(); }

		// Determine # of arguments and file size
		int nArgIn = 0;
    if (E.numElements() == 0)
			nArgIn = 2;
    else if (d.numElements() == 0)
			nArgIn = 3;
		else
			nArgIn = 4;
		int nTotalLengths[4] = {112, 192, 328, 376};
		int nTotalLength = nTotalLengths[nArgIn - 1];

		// Write the file data:
		writeTag(outfile, "CLB ", nTotalLength);

    if (nArgIn > 1) {
			// Write K
      if (K.numCols() != 3 || K.numRows() != 3) { stringstream ssErr; ssErr << "Error in CLB_IO::writeCLB(): Intrinsics matrix was not of dimension 3 by 3."; throw ssErr.str().c_str(); }
			Matrix P = Matrix::eye(3, 4).transpose();
			Matrix K_transposed = K.transpose();
			writeTag(outfile, "PROJ", 3*4*8);
			outfile.write((char *)(double *)P, 3*4*8);
			writeTag(outfile, "INTR", 3*3*8);
			outfile.write((char *)(double *)K_transposed, 3*3*8);
		}
    if (nArgIn > 2) {
			// Write E
      if (E.numCols() != 4 || E.numRows() != 4) { stringstream ssErr; ssErr << "Error in CLB_IO::writeCLB(): Extrinsics matrix was not of dimension 4 by 4."; throw ssErr.str().c_str(); }
			Matrix E_transposed = E.transpose();
			writeTag(outfile, "EXTR", 4*4*8);
			outfile.write((char *)(double *)E_transposed, 4*4*8);
		}
    if (nArgIn > 3) {
			// Write d
      if (d.numElements() != 5) { stringstream ssErr; ssErr << "Error in CLB_IO::writeCLB(): Distortion Coeffs. vector was not of length 5."; throw ssErr.str().c_str(); }
			writeTag(outfile, "DIST", 5*8);
			outfile.write((char *)(double *)d, 5*8);
		}

		// Close the file
		outfile.close();
	}

  void readCLB(const char * fname, Matrix & matIntrinsics, Matrix & matExtrinsics) {
		// TODO: implement more generally.  Currently only works for CLB files w/ intrinsics and extrinsics.  No distortion coeff support

		// Open the file
		ifstream infile(fname, ios::in | ios::binary);
    if (infile.fail()) { stringstream ssErr;  ssErr << "Unable to open file: " << fname; throw ssErr.str().c_str(); }

		// Read in the file
		bool bKRead = false;
		bool bERead = false;
		char chunkHeader[5];
		chunkHeader[4] = '\0';
		int chunkSize;
		double K[3*3]; // 3x3 intrinsics
		double E[4*4]; // 4x4 extrinsics

    while (! bKRead || ! bERead) {
			infile.read(chunkHeader, 4);
			infile.read((char *)&chunkSize, 4);

      if (strcmp(chunkHeader, "CLB ") == 0) {
				// NOP
			}
      else if (strcmp(chunkHeader, "INTR") == 0) {
				// Read in the 3x3 double matrix of intrinsics K (row-major order):
				infile.read((char *)K, 3*3*8);
				bKRead = true;
			}
      else if (strcmp(chunkHeader, "EXTR") == 0) {
				// Read in the 4x4 double matrix of extrinsics E (row-major order):
				infile.read((char *)E, 4*4*8);
				bERead = true;
			}
      else {
				// seek past chunkSize
				infile.seekg(chunkSize, ios::cur);
			}
		}

		// Close the file
		infile.close();

		// Massage data into matExtrinsics and matIntrinsics
		matIntrinsics = Matrix(K, 3*3);
		matExtrinsics = Matrix(E, 4*4);
		matIntrinsics.reshape(3, 3);
		matExtrinsics.reshape(4, 4);
		matIntrinsics = matIntrinsics.transpose(); // convert to column-major
		matExtrinsics = matExtrinsics.transpose(); // convert to column-major
	}

}

#endif
