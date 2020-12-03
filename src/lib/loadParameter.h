#ifndef _LOADPARAMETER_H_
#define _LOADPARAMETER_H_
// basic file operations

/*
* Parameter loading library 
* Modifier - Weiye Zhao
* Date - Nov. 5, 2019
*/

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "structure.h"
using namespace std;


int loadParameter(std::vector<int>& par){
    fstream myfile;
    par.clear();
    myfile.open ("parameter/parameter.txt");
    int data;
    while (!myfile.eof()){
        myfile >> data;
        par.push_back(data);
        // std::cout << data << endl;
    }
    myfile.close();
    return 0;
}

int loadAlgorithm(){
    fstream myfile;
    myfile.open ("parameter/algorithm.txt");
    int data;
    myfile >> data;
    myfile.close();
    return data;
}

int loadMaxIt(){
    fstream myfile;
    myfile.open ("parameter/algorithm.txt");
    int data;
    myfile >> data >> data;
    myfile.close();
    return data;
}

int loadUref(std::vector<double>& par){
    fstream myfile;
    par.clear();
    myfile.open ("parameter/uref.txt");
    double data;
    while (!myfile.eof()){
        myfile >> data;
        par.push_back(data);
        // std::cout << data << endl;
    }
    myfile.close();
    return 0;
}

int loadXref(std::vector<double>& par){
    fstream myfile;
    par.clear();
    myfile.open ("parameter/xref.txt");
    double data;
    while (!myfile.eof()){
        myfile >> data;
        par.push_back(data);
        // std::cout << data << endl;
    }
    myfile.close();
    return 0;
}

int loadXrefPosition(std::vector<double>& par){
    fstream myfile;
    par.clear();
    myfile.open ("ref_new.txt");
    double data;
    while (!myfile.eof()){
        myfile >> data;
        par.push_back(data);
        // std::cout << data << endl;
    }
    myfile.close();
    return 0;
}


void setInputField(inputfield& line, string& data){
    /*
    * make sure each txt line has no ' ' in the end
    * make sure there is '\n' at end of txt file 
    */
    // words count 
    int cnt = 0;
    string word = ""; 
    for (auto x : data){
        if (x == ' ') 
       {    
            if(cnt == 0){
                // get field
                line.field.assign(word);
                word = ""; // empty word
                cnt++;
            }
            else{
                // get elements
                // cout << word << endl;
                line.elements.push_back(stof(word)); // push back next element in float
                word = ""; // empty word
                cnt++;
            }
       } 
       else
       { 
           word = word + x; 
       } 
    }
    // the last element doesn't have blanket behind it
    // cout << word << endl;
    line.elements.push_back(stof(word)); // push back next element in float

    // print out to test the correctness
    // cout << line.field << endl;
    // for (vector<float>::const_iterator i = line.elements.begin(); i != line.elements.end(); ++i)
    //     cout << *i << ' ';
    // cout << endl <<  "good to go" << endl;
}


int loadPlanningProblemSetting(int& njoint, int& nstate, int& nu, double& margin, int& neq, MatrixXd& base, vector<double>& weight_ref, vector<double>& weight_self){
    ifstream file("parameter/parameters.txt");
    string data;
    inputfield line;
    while (std::getline(file, data)) {
        // cout << data << "\n";
        // clear line 
        line.field.clear();
        line.elements.clear();
        
        // customized parameter setting
        // get the elements in string
        // the first elements is name field
        setInputField(line, data);

        // set the input value
        if (line.field.compare("njoint") == 0){
            njoint = (int) line.elements[0];
            cout << "njoint is :" << njoint << endl;
        }
        if (line.field.compare("nstate") == 0){
            nstate = (int) line.elements[0];
            cout << "nstate is :" << nstate << endl;
        }
        if (line.field.compare("nu") == 0){
            nu = (int) line.elements[0];
            cout << "nu is :" << nu << endl;
        }
        if (line.field.compare("margin") == 0){
            margin = (double) line.elements[0];
            cout << "margin is :" << margin << endl;
        }
        if (line.field.compare("neq") == 0){
            neq = (int) line.elements[0];
            cout << "neq is :" << neq << endl;
        }
        if (line.field.compare("base") == 0){
            base << (double) line.elements[0],
                    (double) line.elements[1],
                    (double) line.elements[2];
            cout << "base is :" << base << endl;
        }
        if (line.field.compare("weight_ref") == 0){
            cout << "weight_ref elements:" << endl;
            for (vector<float>::const_iterator i = line.elements.begin(); i != line.elements.end(); ++i){
                cout << (double) *i << endl;
                weight_ref.push_back((double) *i);
            }
        }
        if (line.field.compare("weight_self") == 0){
            cout << "weight_self elements:" << endl;
            for (vector<float>::const_iterator i = line.elements.begin(); i != line.elements.end(); ++i){
                cout << (double) *i << endl;
                weight_self.push_back((double) *i);
            }
        }
    }
    cout << "all seems good" << endl;
    // abort();
    return 0;
}

// checking functions
void print_vector(std::vector<double> v){
    for(double n : v) {
        std::cout << n << '\n';
    }
}

#endif