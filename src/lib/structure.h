#ifndef _Structure_
#define _Structure_

/*
* Basic setups 
* specify all the referenced strcture
* Author - Jessica Leu
* Date - Dec. 3, 2020
*/

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <iostream>


using namespace Eigen;
using namespace std;

struct capsule{
    MatrixXd p;
    double r;
};

struct lineseg{
    MatrixXd p1;
    MatrixXd p2;
};

struct inputfield{
	string field;
	vector<float> elements;
};

#endif
