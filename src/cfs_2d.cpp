/*
* Main function of the CFS 2D planning example. 
* Author - Jessica Leu
* Date - Dec. 3, 2020
*/

#include <cstdio>
#include <ctime>
#include <chrono>
#include <sstream>
#include <string>
#include <string>
#include <fstream>
#include <iterator>
#include <array>
#include "robot_property_eigen.h"
#include "QuadProg++.hh"
#include "CFS.h"
#include "MPsolver.h"
#include "structure.h"

using namespace std;
using namespace Eigen;

int main() {
    /*
    * robot property initialization, use robot type number specification
    * 1 - Turtlebot3    
    */
    //cout << "ayaya" << endl;
    string name = "Turtlebot3"; 
    Robot robot(name);  //"robot_property_eigen.h"
    // CFS planning problem initialization
    CFSProblem pp(robot);    
    // Solve motion planning problem   
    MPsolver cfs(&pp);
    // start execution     
    cfs.iteration(pp.max_iter, pp.converge_trh, robot);    

    // print results
    cfs.printResult();

    /*
    Save the reference trajectory solution
    */
    std::ofstream ofs;
    ofs.open("xreference_2d.txt", std::ofstream::out | std::ofstream::trunc);
    ofs.close();
    std::ofstream outfile;
    outfile.open("xreference_2d.txt", std::ios_base::app);
    outfile << "\n";
    for (int i=0; i<cfs.soln_.rows(); ++i){
        outfile << cfs.soln_(i,0);
        outfile << "\n";
    }
    /*
    Save environment (obstacles)
    */
    std::ofstream ofs2;
    ofs2.open("info.txt", std::ofstream::out | std::ofstream::trunc);
    ofs2.close();
    std::ofstream outfile2;
    outfile2.open("info.txt", std::ios_base::app);
    outfile2 << "\n";
    for (int i=0; i<pp.obs_.cols(); ++i){
        outfile2 << pp.obs_(0,i);
        outfile2 << ',';
        outfile2 << pp.obs_(1,i);
        outfile2 << "\n";
    }

}


