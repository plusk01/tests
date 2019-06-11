/**
 * Linear Program (LP) example, solved using CSDP.
 * See slide 16 of https://ocw.mit.edu/courses/sloan-school-of-management/15-053-optimization-methods-in-management-science-spring-2013/tutorials/MIT15_053S13_tut01.pdf
 * 
 *      max z = 45p + 60q + 50r
 *      st      20p + 10q + 10r <= 2400
 *              12p + 28q + 16r <= 2400
 *              15p +  6q + 16r <= 2400
 *              10p + 15q       <= 2400
 *                       0 <= p <= 100
 *                       0 <= q <= 40
 *                       0 <= r <= 60
 *
 *  The optimal solution is: p* = 81.82; q* = 16.36; r* = 60.
 *  The objective is z = 7664.
 *  
 *  We can recast this LP as the following SDP problem (which CSDP expects):
 *  
 *      max tr(CX)
 *      st  tr(Ai*X) = bi, forall i \in [m]
 *          X >= 0 (p.s.d)
 *
 *  where
 *  
 *      C  = diag([45 60 50])
 *      
 *    constraints:
 *      A1 = diag([20 10 10])
 *      A2 = diag([12 28 16])
 *      A3 = diag([15  6 16])
 *      A4 = diag([10 15  0])
 *
 */
#include <iostream>
#include <string>
#include <vector>

#include <declarations.h>

int main(int argc, char** argv)
{

  struct blockmatrix C;
  double *b;
  struct constraintmatrix *constraints;

  return 0;
}
