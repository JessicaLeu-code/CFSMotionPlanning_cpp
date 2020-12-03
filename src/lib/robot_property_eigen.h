#ifndef _ROBOT_H_
#define _ROBOT_H_

/*
* Robot property class defintion 
* Robot model: point mass (Turtlebot3)
* Author - Jessica Leu
* Date - Dec. 3, 2020
*/

#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include "structure.h"
#include <string>
using namespace std;
using namespace Eigen;


class Robot {

    public:
    int nlink;
    int umax;
    double margin;
    double pi;
    double delta_t;
    int nu_;
    int nstate_;
    string name;
    MatrixXd thetamax;
    MatrixXd thetadotmax;
    MatrixXd l;
    MatrixXd DH;
    MatrixXd base;
    MatrixXd A;
    MatrixXd B;
    MatrixXd Ac;
    MatrixXd Bc;
    capsule cap[6];
    lineseg pos[6];
    MatrixXd M[7];



  Robot(string robot_name){
    int robot_num = parse_name(robot_name);
    switch (robot_num){
        case 1: {
            Turtlebot3_base();

            // xy integrator
            A.resize(2,2);
            A << MatrixXd::Identity(2,2);

            B.resize(2,2);
            B << MatrixXd::Identity(2,2) * delta_t;
            break;
            
            
        }
        
    }
    
  }


    int parse_name(string name){
        string gp50, M16iB, tb3;
        gp50 = "gp50";
    M16iB = "M16iB";
    tb3 = "Turtlebot3";
    if((name.compare(tb3)) == 0){
        cout << "robot is identified as " << tb3 << endl; 
        cout << "------------ start initialize robot property ------------" << endl;
        return 1;
    }


    else{
        cout << "no match robot property found" << endl;
        abort();
    } 
    return 0;
  }

  void Turtlebot3_base(){
    name = "TB3";
    delta_t = 0.2;
    nu_ = 2;
    nstate_ = 2;
  }  


};
#endif
