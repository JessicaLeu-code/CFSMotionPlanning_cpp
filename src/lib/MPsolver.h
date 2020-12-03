#ifndef _CFSSOLVER_H_
#define _CFSSOLVER_H_
/*
* Solve motion planning problem
* (convert the nominal CFS planning problem to be compatible with the QP solver) 
* Author - Jessica Leu
* Date - Dec. 3, 2020
*/

#include "customizedPrint.h"
#include "CFS.h"
#include <math.h>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include "QuadProg++.hh"
#include "robot_property_eigen.h"
#include "math_pack.h"
#include "structure.h"

#include "matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;


using namespace std;

class MPsolver {


    public:
        // Problem type
        CFSProblem* pp_;
        
        // store results
        MatrixXd soln_;
        MatrixXd xold_;
        MatrixXd xR_; // xref is the same as xR, by stacking each column of xR;
        
        // QuadProg++.hh solver info
        /*
        min 0.5 * x G x + g0 x
        s.t.
            CE^T x + ce0 = 0
            CI^T x + ci0 >= 0     
        */ 
        quadprogpp::Vector<double> u;
        quadprogpp::Matrix<double> G, CE, CI;
        quadprogpp::Vector<double> g0, ce0, ci0;
        // qp temp
        MatrixXd temp_A, temp_Aeq;

        // solver statistics
        double iteration_time_;
        int iteration_;
        double tmp_cost;
        vector<double> qp_time_;
        vector<double> process_time_;
        vector<double> cost_;    


    MPsolver(CFSProblem* pp) {
        pp_ = pp;
    }

    int iteration(int iterMax, double tolX, Robot& robot){        

        // record time
        double iter_start_time = 0;
        double iter_end_time = 0;
        double qp_start_time = 0;
        double qp_end_time = 0;
        double process_start_time = 0;
        double process_end_time = 0;

        // The iteration start
        qp_time_.clear();
        cost_.clear();
        iter_start_time = clock();     // record time
        printMessage("solving CFS...");

        for (int k=0; k<iterMax; k++){            
            cout << "----------------------" << endl;
            cout << "Iteration " << k << endl;
            // initialize reference input u
            process_start_time = clock(); 
            QPxset();
            // store old solution
            pp_->xold_ = pp_->xref_;
            pp_->uold_ = pp_->uref_;
            // set QP cost
            setQPObjValue();            
            // set linear inequality constraint          
            pp_->get_gamma(robot.delta_t);
            // CI = -Ainq_', ci0 = binq_            
            temp_A = -1*pp_->Ainq_.transpose(); // negative transpose
            setConstraint(CI, ci0, temp_A, pp_->binq_);
            // set linear equality constraint 
            temp_Aeq = -1*pp_->Aeq_.transpose();
            setConstraint(CE, ce0, temp_Aeq, pp_->beq_);

            process_end_time = clock();
            process_time_.push_back((process_end_time - process_start_time)/CLOCKS_PER_SEC);

            // Solve the subproblem                  
            qp_start_time = clock();
            tmp_cost = solve_quadprog(G, g0, CE, ce0, CI, ci0, u);
            cost_.push_back(tmp_cost*2+(double)pp_->c_(0,0));         
            quad2matrix(u, pp_->uref_); // update uref_ in planning problem;
            
            // clock stop
            qp_end_time = clock();
            qp_time_.push_back((qp_end_time - qp_start_time)/CLOCKS_PER_SEC);
            cout << "cost temporal cost is: " << tmp_cost << endl;

            // get new xref_
            pp_->get_xref();
            //cout<<pp_->xref_<<endl;

            //Convergence check
            if (checkConvergence(pp_->xref_,pp_->xold_,tolX)){
                cout << "Converged at step " << k+1 << endl;
                iteration_ = k;
                break;
            }   
        }
        soln_ = pp_->xref_;
        iter_end_time = clock();
        iteration_time_ = (iter_end_time - iter_start_time)/CLOCKS_PER_SEC;
        plotxref();          

        return 0;
    }

    void QPxset(){
        u.resize(pp_->uref_.rows());
        for (int i = 0; i < pp_->uref_.size(); i++) 
            u[i] = pp_->uref_(i,0);
    }
    void setQPObjValue(){
        int Hn_, fn_;
        Hn_ = pp_->H_.rows();
        fn_ = pp_->f_.rows();
        // H_ to G
        G.resize(Hn_, Hn_);
        for (int i = 0; i < Hn_; i++) 
            for (int j = 0; j < Hn_; j++)
                G[i][j] = pp_->H_(i,j);
        // f_ to g0
        g0.resize(fn_);
        for (int i = 0; i < fn_; i++) 
            g0[i] = pp_->f_(i,0);
    }

    void setConstraint(quadprogpp::Matrix<double> &C, quadprogpp::Vector<double> &c0, const MatrixXd& LT, const MatrixXd& S){    
        int Lnr_ = LT.rows();
        int Lnc_ = LT.cols();
        int Sn_ = S.rows();
        C.resize(Lnr_, Lnc_);
        for (int i = 0; i < Lnr_; i++) 
            for (int j = 0; j < Lnc_; j++)
                C[i][j] = LT(i,j);
        c0.resize(Sn_);
        for (int i = 0; i < Sn_; i++) 
            c0[i] = S(i,0);
    }


    bool checkConvergence(const MatrixXd& x, MatrixXd& xold, const double& tolX){
        MatrixXd diff;        
        diff = x - xold;        
        double norm_diff;
        norm_diff = diff.norm();
        if (norm_diff < tolX){
            return true;
        }
        else{
            return false;
        }
    }
    void printResult(){
        cout << "the process time (second) is:";
        printVector(process_time_.data(),process_time_.size());
        cout << "the QP time (second) is:";
        printVector(qp_time_.data(),qp_time_.size());
        cout << "total iteration time = " << iteration_time_ << "second" << endl;
    }

    void plotxref(){
        
        int n = pp_->nstep_;
        int dim = pp_->dim_;
        int resolution = 50;
        //plot trajectory
        std::vector<double> x(n), y(n), obs_x(resolution+1),obs_y(resolution+1);
        for(int i=0; i<n; ++i) {
            x.at(i) = pp_->xref_(i*dim,0);
            y.at(i) = pp_->xref_(i*dim+1,0);            
        } 
        plt::figure_size(800, 800);        
        plt::named_plot("Planned trajectory",x, y);
        // plot obstacle
        for(int j = 0; j<pp_->nobs_; j++){
            for(int i=0; i<resolution+1; ++i) {
                obs_x.at(i) = cos(M_PI*2*i/resolution)*pp_->obs_(2,j)+pp_->obs_(0,j);
                obs_y.at(i) = sin(M_PI*2*i/resolution)*pp_->obs_(2,j)+pp_->obs_(1,j);                      
            }
            plt::plot(obs_x, obs_y,"r");
        }      
        // plot setups
        plt::xlim(-0.2, 1.0);
        plt::ylim(-0.6, 0.6);
        plt::legend();
        plt::show();
    }

};

#endif
