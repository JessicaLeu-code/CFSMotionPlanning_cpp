#ifndef _CFS_H_
#define _CFS_H_

/*
* CFS class defintion 
* Author - Jessica Leu
* Date - Dec. 3, 2020
*/

#include "customizedPrint.h"
#include "supportfunctions.h"
#include "loadParameter.h"
#include "robot_property_eigen.h"
#include "structure.h"
#include "math_pack.h"
using namespace std;

// Motion Planning Problem
/*
min u'*H*u + 2*f'*u + c
s.t.
    Ainq*u <= bieq
    Aeq*u = beq 
*/
class CFSProblem {    
    public:
        // Setups
        int nstep_;             // number of waypoints (include current position)
        int nstate_;            // number of full state
        int dim_;               // dimension of the cartesian space
        int nu_;                // number of inputs
        int horizon_;           // number of planning horizon
        int nn_;                // number of decision variables (nn = nu * horizon)

        //Kinematic matrix
        /*
          x = Aaug_*x0 + Baug_*u
        */
        MatrixXd x0_;            // [nstate x 1]
        MatrixXd xg_;
        MatrixXd Aaug_;          // [nstate*horizon x nstate]
        MatrixXd Baug_;          // [nstate*horizon x nn]
        vector<double> weight_self_;
        double v_max;

        // Obstacles
        int nobs_ = 1;
        double margin;
        MatrixXd obs_;
        // reference
        vector<double> xref_vec; 
        MatrixXd xref_;        
        MatrixXd uref_;
        bool UseRef = false;    

        // Cost function
        MatrixXd H_;            // quadratic   (u'*H_*u) 
        MatrixXd f_;            // linear      (f_'*u)
        MatrixXd c_;            // constant            
        MatrixXd goal_;         // goal vecore (||x-goal_||^2)  

        // Constraints
        // equaliry constraints
        // inequality contraints
        MatrixXd Ainq_, binq_;   // inequality constraints (Lfull_* u < S_)
        MatrixXd Aeq_, beq_;     // equality constraints

        // CFS solver setup
        int max_iter;
        double converge_trh;    
        MatrixXd uold_;
        MatrixXd xold_;

    CFSProblem(Robot& robot){
        // Robot info setup
        nu_ = robot.nu_;
        nstate_ = robot.nstate_;
        dim_ = nstate_;    
        // Obs setup   
        loadobs();    
        // Reference waypoint setup        
        gen_xref(xref_vec,UseRef);                 
        setReferenceTrajectory();        
        x0_.resize(nstate_,1);        
        x0_<<xref_.topRows(nstate_);
        xg_.resize(nstate_,1);        
        xg_<<xref_.bottomRows(nstate_);
        goal_.resize((int)xref_.rows()-nstate_,1);
        for(int i=0; i<horizon_;i++){
            for(int j = 0; j<nstate_;j++){
                goal_(i*nstate_+j,0)=xg_(j,0);
            }
        }

        // Other parameter setup        
        loadProblemSetting(max_iter, converge_trh, margin, v_max, weight_self_);              
        nstep_= (int) xref_.rows() / dim_; // planning horizon        
        horizon_ = nstep_-1;
        nn_ = horizon_*nu_;        

        // Initialize u0
        uref_ = MatrixXd::Constant(nn_,1,0);
         
        // Aaug Baug setup
        get_ABaug(robot);     
       
        // Cost function setup
        set_cost();

        // Constraints setup
        Ainq_.resize(horizon_*nobs_+nn_*2,nn_);
        binq_.resize(horizon_*nobs_+nn_*2,1);
        Ainq_<<MatrixXd::Zero(horizon_*nobs_+nn_*2,nn_);
        binq_<<MatrixXd::Zero(horizon_*nobs_+nn_*2,1);
        get_gamma(robot.delta_t);
        Aeq_.resize(1,nn_);
        beq_.resize(1,1);
        Aeq_<<MatrixXd::Zero(1,nn_);
        beq_<<MatrixXd::Zero(1,1);
        Aeq_(0,horizon_*2-1)=1;
        cout<<"CFS problem all set."<<endl;      

        
    }    

    void get_xref(){
        xref_ << x0_,Aaug_*x0_+Baug_*uref_; 
    }

    void get_ABaug(Robot& robot){
        Aaug_.resize(horizon_*nstate_,nstate_);
        Baug_.resize(horizon_*nstate_,nn_);
        for (int i=0; i<horizon_; i++){            
            Aaug_.block(i*nstate_,0,2,2)=robot.A;
            for (int j=0; j<=i;j++){
                Baug_.block(i*nstate_,j*nstate_,2,2)=robot.B;
            }
        }  
    }

    void set_cost(){
        // quadratic cost                             
        int Q_size = horizon_*nstate_; 
        MatrixXd Q_(Q_size, Q_size);
        Q_ << MatrixXd::Identity(Q_size-nstate_, Q_size-nstate_)*10, MatrixXd::Zero(Q_size-nstate_, nstate_),
                MatrixXd::Zero(nstate_, Q_size-nstate_), MatrixXd::Identity(nstate_, nstate_)*50;        
        int R_size = nn_; 
        MatrixXd R_(R_size, R_size);
        R_ << MatrixXd::Identity(R_size, R_size);
        H_ = Baug_.transpose()*Q_*Baug_+weight_self_[0]*R_;        
        // linear cost 
        MatrixXd temp_f = -(-x0_.transpose()*Aaug_.transpose()+goal_.transpose())*Q_*Baug_;
        f_ = temp_f.transpose();         
        // constant cost
        MatrixXd temp_c = (-x0_.transpose()*Aaug_.transpose()+goal_.transpose());
        c_ = temp_c*Q_*temp_c.transpose();       

    }

    void get_gamma(double delta_t){
        MatrixXd temp_a=MatrixXd::Zero(2,nstate_); 
        MatrixXd temp_b=MatrixXd::Zero(2,nn_);

        MatrixXd x(2,1);
        MatrixXd gx(1,1);
        MatrixXd dgx(2,1);
        // constraints avoiding obstacles 
        for (int j = 0; j<nobs_;j++){                        
            for (int i=0; i<horizon_; i++){
                x << xref_.middleRows(i*nstate_, nstate_);  
                // g(x*)+dg(x*)'(Ax0+Bu -x*)<=0                
                gx << g_f(obs_.block(0, j, 2, 1),obs_(2,j),x);
                dgx << dg_f(obs_.block(0, j, 2, 1),x);
                
                temp_a.row(0)= Aaug_.row(i*nstate_);
                temp_a.row(1)= Aaug_.row(i*nstate_+1);
                temp_b.row(0)= Baug_.row(i*nstate_);
                temp_b.row(1)= Baug_.row(i*nstate_+1);

                Ainq_.row(j*horizon_+i) = dgx.transpose()*temp_b;            
                binq_.row(j*horizon_+i) = dgx.transpose()*x-gx-dgx.transpose()*temp_a*x0_;                                
            }
        }
        // velocity constraints 
        Ainq_.bottomRows(nn_*2) << MatrixXd::Identity(nn_,nn_),-MatrixXd::Identity(nn_,nn_);
        binq_.bottomRows(nn_*2) << MatrixXd::Constant(nn_*2, 1, v_max*delta_t);
        
    }

    int gen_xref(std::vector<double>& xref_vec, bool UseRef){
        // Load whole trajectory
        fstream myfile;
        xref_vec.clear();
        if (UseRef){            
            myfile.open ("ref_new.txt");
            double data;
            while (!myfile.eof()){
                myfile >> data;
                xref_vec.push_back(data);
                // std::cout << data << endl;
            }
            myfile.close();
        }else{
        /* ref_init_goal.txt
           0.7    initial x position 
           0.0    initial y position
           0.0    goal x position
           0.0    goal y position
           24     planning horizon
        */           
            myfile.open ("ref_init_goal.txt");
            double data;
            std::vector<double> init_goal;
            int count =0;
            while (count<5){
                myfile >> data;
                init_goal.push_back(data);                
                count ++;                
            }            
            myfile.close();
            
            // interpolation
            horizon_ = init_goal.back();           
            double dx = (init_goal[2] - init_goal[0])/horizon_;
            double dy = (init_goal[3] - init_goal[1])/horizon_;
            xref_vec.push_back(init_goal[0]);
            xref_vec.push_back(init_goal[1]);            
            for(int i=1; i<horizon_+1;i++){
                xref_vec.push_back(init_goal[0]+i*dx);
                xref_vec.push_back(init_goal[1]+i*dy);                
            }
            cout << "Initial position is: " << "("<<init_goal[0]<< ","<<init_goal[1]<<")"<< endl;        
            cout << "Goal position is: " << "("<<init_goal[2]<< ","<<init_goal[3]<<")"<< endl;

        }
        cout << "The plannig horizon is: " << horizon_<< endl;

        return 0;
    }       

    void setReferenceTrajectory(){
        xref_.resize(xref_vec.size(),1);
        for (int i=0; i<xref_vec.size(); ++i){
            xref_(i,0) = xref_vec[i];
        }
    }
    int loadobs(){
        fstream myfile;
        /* parameter/env.txt
           2       number of obstacles
           0.3     obs1 x location 
           0.01    obs1 y location
           0.1     obs1 radius
           0.5     obs2 x location 
           -0.01   obs2 y location
           0.1     obs2 radius
           .
           .
           .
        */                      
        myfile.open ("parameter/env.txt");
        double data;
        myfile >> data;
        nobs_ = (int)data;
        obs_.resize(3,nobs_);
        int i = 0;        
        while (!myfile.eof()){
            myfile >> data;
            obs_(i%3,i/3) = data;
            i++;
        }
        cout<< "obs1, obs2, ... ([x;y;radius])"<<endl;
        std::cout << obs_ << endl;
            
        myfile.close();
        return 0;
    }

    int loadProblemSetting(int& max_iter, double& converge_trh, double& margin, double& v_max, vector<double>& weight_self){
        ifstream file("parameter/parameters_2d.txt");
        string data;
        inputfield line;
        while (std::getline(file, data)) {            
            // clear line 
            line.field.clear();
            line.elements.clear();
            
            // customized parameter setting
            // get the elements in string
            // the first elements is name field
            setInputField(line, data);

            // set the input value
            if (line.field.compare("max_iter") == 0){
                max_iter = (int) line.elements[0];
                cout << "max_iter is :" << max_iter << endl;
            }
            if (line.field.compare("converge_trh") == 0){
                converge_trh = (double) line.elements[0];
                cout << "convergence threshold is :" << converge_trh << endl;
            }
            if (line.field.compare("margin") == 0){
                margin = (double) line.elements[0];
                cout << "margin is :" << margin << endl;
            }
            if (line.field.compare("v_max") == 0){
                v_max = (double) line.elements[0];
                cout << "Vmax is :" << v_max << endl;
            }
            if (line.field.compare("weight_self") == 0){
                cout << "weight_self elements: ";
                for (vector<float>::const_iterator i = line.elements.begin(); i != line.elements.end(); ++i){
                    cout << (double) *i << " ";
                    weight_self.push_back((double) *i);
                }
                cout<<endl;
            }
        }
        return 0;
    }
    

};
#endif