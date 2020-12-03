#ifndef _SUPPORTFUNCTIONS_H_
#define _SUPPORTFUNCTIONS_H_
/*
* 2D distance function library 
* Modifier - Jessica Leu
* Date - Dec. 3, 2020
*/

#include <iostream>
#include <fstream>
#include <stdio.h>      
#include <math.h>      
#include "structure.h"

using namespace std;

MatrixXd g_f(MatrixXd c, double m, MatrixXd x){
	MatrixXd g(1,1);
	double t6, t7, t8, t9;	
	t6 = -x(0);
	t7 = -x(1);
	t8 = c(0)+t6;
	t9 = c(1)+t7;
	x.transpose();
	g << -t8*(c(0)-x(0))-t9*(c(1)-x(1))+pow(m,2.0);
	//double d = x.transpose()*x-2*x.transpose()*c+c.transpose()*c;
    //g<<pow(m,2.0)-d ;
    return g;
}

MatrixXd dg_f(MatrixXd c, MatrixXd x){
	MatrixXd dg(2,1);
	double t10, t11, t12, t13;
		
	t10 = c(0)-x(0);
	t11 = c(1)-x(1);
	t12 = c(0)-x(0);
	t13 = c(1)-x(1);
	//dg<< t10+t12, t11+t13;
    dg<< -2*x+2*c;
    return dg;
}


#endif
