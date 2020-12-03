#ifndef _MathPack_
#define _MathPack_

/*
* Math operation library for CFS   
* Author - Weiye Zhao
* Date - Nov. 5, 2019
*/

#include <Eigen/Dense>
#include "structure.h"
#include "QuadProg++.hh"
#include "robot_property_eigen.h"
#include <cmath>
#include <vector>
#include <chrono> 

using namespace Eigen;
using namespace std;
using namespace std::chrono;


void quad2matrix(quadprogpp::Vector<double> u, MatrixXd& u_){
  int row = u_.rows();
  for(int i=0; i<row; ++i){
    u_(i,0) = u[i];
  }
} 

MatrixXd MatPower(MatrixXd& mat, int power){
  MatrixXd new_mat;
  new_mat = mat;
  for (int i=0; i<power-1; ++i){
    new_mat = new_mat * mat;
  }
  if (power == 0)
    new_mat = MatrixXd::Identity(mat.rows(),mat.cols());
  return new_mat;
}

double pow_sum(MatrixXd m, int pwr){
  assert(m.cols() == 1);
  double sum = 0;
  MatrixXd tmp;
  tmp = m.array().pow(pwr);
  for (int i=0; i<m.rows(); ++i){
    sum += tmp(i,0);
  }
  return sum;
}

double dot_sum(MatrixXd m, MatrixXd n){
  assert(m.rows() == n.rows());
  double sum = 0;
  for (int i=0; i<m.rows(); ++i){
    sum += m(i,0) * n(i,0);
  }
  return sum;
}

double fixbound(double num){
  if (num < 0){
    num = 0;
  }
  else{
    if (num > 1){
      num = 1;
    }
  }
  return num;
}

#endif