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
 *          X >= 0 (p.s.d).
 *
 *  First, note that the dual (or, standard inequality form) of the above SDP is
 *  
 *      min b^T x
 *      st  F0 + x1F1 + ... + xmFm >= (p.s.d) 0.
 *      
 *  Our goal will be to write the LP in the dual, and then get back to the primal
 *  problem for CSDP to solve.
 *  
 *  Let b = [45 60 50] and x = [p q r]. Then
 *  
 *      F0 = diag([2400 2400 2400 2400  100 40 60  0 0 0])
 *  
 *      F1 = diag([20 12 15 10 1 0 0 -1 0 0])
 *      F2 = diag([10 28 6 15 0 1 0 0 -1 0])
 *      F3 = diag([10 16 16 0 0 0 1 0 0 -1])
 *
 *  Therefore, the SDP formulation of this LMI (which encodes the LP) is
 *  
 *      max <diag(F0), X>
 *      st  <F1, X> = 45 = b1
 *          <F2, X> = 60 = b2
 *          <F3, X> = 50 = b3
 *          X >= 0 (p.s.d)
 *
 */
#include <iostream>
#include <string>
#include <vector>

extern "C" {
#include <declarations.h>
}

int main(int argc, char** argv)
{

  //
  // The objective data <C,X>
  // 

  struct blockmatrix C;
  C.nblocks = 1; // 1 block diagonal with 10 elements
  C.blocks = reinterpret_cast<struct blockrec *>(std::malloc((1+1)*sizeof(struct blockrec)));
  if (C.blocks == nullptr) {
    std::cerr << "Couldn't allocate storage for C!" << std::endl;
    return 1;
  }

  C.blocks[1].blockcategory = DIAG;
  C.blocks[1].blocksize = 10;
  C.blocks[1].data.vec = reinterpret_cast<double *>(std::malloc((10+1)*sizeof(double)));
  if (C.blocks[1].data.vec == nullptr) {
    std::cerr << "Couldn't allocate storage for block 1 of C!" << std::endl;
    return 2;
  }

  C.blocks[1].data.vec[1] = -2400.0;
  C.blocks[1].data.vec[2] = -2400.0;
  C.blocks[1].data.vec[3] = -2400.0;
  C.blocks[1].data.vec[4] = -2400.0;
  C.blocks[1].data.vec[5] = -100.0;
  C.blocks[1].data.vec[6] = -40.0;
  C.blocks[1].data.vec[7] = -60.0;
  C.blocks[1].data.vec[8] =  0.0;
  C.blocks[1].data.vec[9] =  0.0;
  C.blocks[1].data.vec[10] = 0.0;

  //
  // RHS of constraints, b
  //

  double *b = reinterpret_cast<double *>(std::malloc((3+1)*sizeof(double)));
  if (b == nullptr) {
    std::cerr << "Failed to allocate storage for RHS constraints, b!" << std::endl;
    return 3;
  }

  b[1] = 45.0;
  b[2] = 60.0;
  b[3] = 50.0;

  //
  // Constraints, Ai
  //

  // pointer to blocks in constraint matrices
  struct sparseblock *blockptr;

  struct constraintmatrix *constraints = reinterpret_cast<struct constraintmatrix *>(
                                        std::malloc((3+1)*sizeof(struct constraintmatrix)));
  if (constraints == nullptr) {
    std::cerr << "Failed to allocate storage for constraint matrices, Ai!" << std::endl;
    return 4;
  }

  //
  // A1
  //

  // build linked list of constraint matrix blocks in reverse order
  constraints[1].blocks = nullptr;

  // block 1 of A1
  blockptr = reinterpret_cast<struct sparseblock *>(std::malloc(sizeof(struct sparseblock)));
  if (blockptr == nullptr) {
    std::cerr << "Allocation of constraint block failed!" << std::endl;
    return 5;
  }

  blockptr->blocknum = 1;
  blockptr->blocksize = 10;
  blockptr->constraintnum = 1;
  blockptr->next = nullptr;
  blockptr->nextbyblock = nullptr;
  blockptr->entries = reinterpret_cast<double *>(std::malloc((6+1)*sizeof(double)));
  blockptr->iindices = reinterpret_cast<int *>(std::malloc((6+1)*sizeof(int)));
  blockptr->jindices = reinterpret_cast<int *>(std::malloc((6+1)*sizeof(int)));
  if (blockptr->entries == nullptr || blockptr->iindices == nullptr || blockptr->jindices == nullptr) {
    std::cerr << "Allocation of constraint block failed!" << std::endl;
    return 6;
  }

  blockptr->numentries = 6;
  blockptr->iindices[1] = blockptr->jindices[1] = 1; blockptr->entries[1] = 20.0;
  blockptr->iindices[2] = blockptr->jindices[2] = 2; blockptr->entries[2] = 12.0;
  blockptr->iindices[3] = blockptr->jindices[3] = 3; blockptr->entries[3] = 15.0;
  blockptr->iindices[4] = blockptr->jindices[4] = 4; blockptr->entries[4] = 10.0;
  blockptr->iindices[5] = blockptr->jindices[5] = 5; blockptr->entries[5] =  1.0;
  blockptr->iindices[6] = blockptr->jindices[6] = 8; blockptr->entries[6] = -1.0;

  blockptr->next = constraints[1].blocks;
  constraints[1].blocks = blockptr;

  //
  // A2
  //

  // build linked list of constraint matrix blocks in reverse order
  constraints[2].blocks = nullptr;

  // block 1 of A2
  blockptr = reinterpret_cast<struct sparseblock *>(std::malloc(sizeof(struct sparseblock)));
  if (blockptr == nullptr) {
    std::cerr << "Allocation of constraint block failed!" << std::endl;
    return 5;
  }

  blockptr->blocknum = 1;
  blockptr->blocksize = 10;
  blockptr->constraintnum = 2;
  blockptr->next = nullptr;
  blockptr->nextbyblock = nullptr;
  blockptr->entries = reinterpret_cast<double *>(std::malloc((6+1)*sizeof(double)));
  blockptr->iindices = reinterpret_cast<int *>(std::malloc((6+1)*sizeof(int)));
  blockptr->jindices = reinterpret_cast<int *>(std::malloc((6+1)*sizeof(int)));
  if (blockptr->entries == nullptr || blockptr->iindices == nullptr || blockptr->jindices == nullptr) {
    std::cerr << "Allocation of constraint block failed!" << std::endl;
    return 6;
  }

  blockptr->numentries = 6;
  blockptr->iindices[1] = blockptr->jindices[1] = 1; blockptr->entries[1] = 10.0;
  blockptr->iindices[2] = blockptr->jindices[2] = 2; blockptr->entries[2] = 28.0;
  blockptr->iindices[3] = blockptr->jindices[3] = 3; blockptr->entries[3] =  6.0;
  blockptr->iindices[4] = blockptr->jindices[4] = 4; blockptr->entries[4] = 15.0;
  blockptr->iindices[5] = blockptr->jindices[5] = 6; blockptr->entries[5] =  1.0;
  blockptr->iindices[6] = blockptr->jindices[6] = 9; blockptr->entries[6] = -1.0;

  blockptr->next = constraints[2].blocks;
  constraints[2].blocks = blockptr;

  //
  // A3
  //

  // build linked list of constraint matrix blocks in reverse order
  constraints[3].blocks = nullptr;

  // block 1 of A3
  blockptr = reinterpret_cast<struct sparseblock *>(std::malloc(sizeof(struct sparseblock)));
  if (blockptr == nullptr) {
    std::cerr << "Allocation of constraint block failed!" << std::endl;
    return 5;
  }

  blockptr->blocknum = 1;
  blockptr->blocksize = 10;
  blockptr->constraintnum = 3;
  blockptr->next = nullptr;
  blockptr->nextbyblock = nullptr;
  blockptr->entries = reinterpret_cast<double *>(std::malloc((5+1)*sizeof(double)));
  blockptr->iindices = reinterpret_cast<int *>(std::malloc((5+1)*sizeof(int)));
  blockptr->jindices = reinterpret_cast<int *>(std::malloc((5+1)*sizeof(int)));
  if (blockptr->entries == nullptr || blockptr->iindices == nullptr || blockptr->jindices == nullptr) {
    std::cerr << "Allocation of constraint block failed!" << std::endl;
    return 6;
  }

  blockptr->numentries = 5;
  blockptr->iindices[1] = blockptr->jindices[1] = 1; blockptr->entries[1] = 10.0;
  blockptr->iindices[2] = blockptr->jindices[2] = 2; blockptr->entries[2] = 16.0;
  blockptr->iindices[3] = blockptr->jindices[3] = 3; blockptr->entries[3] = 16.0;
  blockptr->iindices[4] = blockptr->jindices[4] = 7; blockptr->entries[4] =  1.0;
  blockptr->iindices[5] = blockptr->jindices[5] =10; blockptr->entries[5] = -1.0;

  blockptr->next = constraints[3].blocks;
  constraints[3].blocks = blockptr;

  //
  // Solve Problem
  //
  
  write_prob("prob.dat-s", 10, 3, C, b, constraints);

  struct blockmatrix X, Z;
  double * y;
  double pobj, dobj;

  initsoln(10, 3, C, b, constraints, &X, &y, &Z);
  int ret = easy_sdp(10, 3, C, b, constraints, 0.0, &X, &y, &Z, &pobj, &dobj);

  if (ret == 0) {
    std::cout << "The objective value is " << (dobj+pobj)/2 << std::endl;
  } else {
    std::cerr << "SDP solve failed." << std::endl;
  }

  write_sol("prob.sol", 10, 3, X, y, Z);

  free_prob(10, 3, C, b, constraints, X, y, Z);

  return 0;
}
