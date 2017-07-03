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

#ifndef __LAPACK_DECLARATIONS_H
#define __LAPACK_DECLARATIONS_H

extern "C" {
	// Linear solvers
	// ===============================================================================================

	// Solution of square linear system(s) AX=B, by LU decomp.
	void dgesv_(long *n, long *nrhs, double *A, long *lda, long *iPiv, double *B, long *ldb, long *info);

	// Solves triangular systems AX=B or A**T * X = B
	void dtrtrs_(char *uplo, char *trans, char *diag, long *n, long *nrhs, double *A, long *lda, double *B, long *ldb, long *info);

	// Decompositions
	// ===============================================================================================

	// LU factorization of a general M-by-N matrix A using partial pivoting with row interchanges.
	void dgetrf_(long *m, long *n, double *a, long *lda, long *ipiv, long *info);

	// QR factorization, without pivoting.
	void dgeqrf_(long *m, long *n, double *A, long *lda, double *tau, double *work, long *lwork, long *info);

	// QR With Column Pivoting
	void dgeqp3_(long *m, long *n, double *A, long *lda, long *jpvt, double *tau, double *work, long *lwork, long *info);

	// SVD
	void dgesvd_(char *jobu, char *jobvt, long *m, long *n, double *A, long *lda, double *s, double *U, long *ldu, double *VT, long *ldvt,
		double *work, long *lwork, long *info);


	// Misc. Helper routines
	// ===============================================================================================

	// Fast orthogonal matrix w/ real matrix multiplication routine
	void dormqr_(char *side, char *trans, long *m, long *n, long *k, double *A, long *lda, double *tau, double *C, long *ldc, double *work,
		long *lwork, long *info);

	// Constructs orthogonal matrix from householder transformation vectors / scalars, as returned by LAPACK qr factorization routines
	void dorgqr_(long *m, long *n, long *k, double *A, long *lda, double *tau, double *work, long *lwork, long *info);
} 

#endif
