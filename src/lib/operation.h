/*
* Matrix/Vector operators
* Using QuadProg++
* Author - Jessica Leu
* Date - Aug. 25, 2020
*/
#include <iostream>

using namespace std;


std::ostringstream msg;

quadprogpp::Matrix<double> Mm(quadprogpp::Matrix<double> &A, quadprogpp::Matrix<double> &B){

	register unsigned int i, j, k, l; /* indices */
	quadprogpp::Matrix<double> mul;
	
	unsigned int An = A.nrows();
	unsigned int Am = A.ncols();
	unsigned int Bn = B.nrows();
	unsigned int Bm = B.ncols();

	mul.resize(An, Bm);

	if (Am !=Bn){
		cout<<"Wrong matrix dimention"<<endl;
		throw std::logic_error(msg.str());
	}
	else{
		for(i=0; i<An; i++){
			for(j=0; j<Bm; j++){
				mul[i][j]=0;
				for(k=0; k<Am; k++){
				mul[i][j]+=A[i][k]*B[k][j];
				}
			}
		}

	}
	
	return mul;    
}


quadprogpp::Matrix<double> Madd(quadprogpp::Matrix<double> &A, quadprogpp::Matrix<double> &B){

	register unsigned int i, j; /* indices */
	quadprogpp::Matrix<double> mul;
	
	unsigned int An = A.nrows();
	unsigned int Am = A.ncols();
	unsigned int Bn = B.nrows();
	unsigned int Bm = B.ncols();

	mul.resize(An, Am);
	
	if (An !=Bn || Am !=Bm ){
		cout<<"Wrong matrix dimention"<<endl;
		throw std::logic_error(msg.str());
	}
	else{
		for(i=0; i<An; i++){
			for(j=0; j<Bm; j++){
				mul[i][j]=A[i][j]+B[i][j];
			}

		}
	}
	
	return mul;    
}


double Vdot(quadprogpp::Vector<double>& x, quadprogpp::Vector<double>& y)
{
  register int i, n = x.size();
  register double sum;
	
  sum = 0.0;
  for (i = 0; i < n; i++)
    sum += x[i] * y[i];
  return sum;			
}

quadprogpp::Vector<double> Vadd(quadprogpp::Vector<double>& v1, quadprogpp::Vector<double>& v2)
{
  register unsigned int i; /* indices */
  quadprogpp::Vector<double> vv;
  register int v1n = v1.size();
  register int v2n = v2.size();
  
	
  if (v1n !=v2n){
		cout<<"Wrong matrix dimention"<<endl;
		throw std::logic_error(msg.str());
	}
	else{
		vv.resize(v1n);
		for(i=0; i<v1n; i++){			
			vv[i]=v2[i]+v1[i];		

		}
	}

	return vv;	
}

double VtMV(quadprogpp::Vector<double>& x,  quadprogpp::Matrix<double> &Q)
{
  quadprogpp::Vector<double> vv;
  register double sum;
  register int  j, k, n = x.size();
  register unsigned int Qn = Q.nrows();
  register unsigned int Qm = Q.ncols();
  
	
  sum = 0.0;

  	if (Qn !=Qm || Qm!=n ){
		cout<<"Wrong matrix dimention"<<endl;
		throw std::logic_error(msg.str());
	}
	else{
		vv.resize(n);		
		for(j=0; j<n; j++){
			vv[j]=0;
			for(k=0; k<n; k++){
			vv[j]+=x[k]*Q[k][j];
			}
			sum += vv[j] * x[j];
		}		
		
	}
  
  return sum;			
}

quadprogpp::Vector<double> MV(quadprogpp::Matrix<double> &Q, quadprogpp::Vector<double>& x)
{
  quadprogpp::Vector<double> vv;  
  register int  j, k, n = x.size();
  register unsigned int Qn = Q.nrows();
  register unsigned int Qm = Q.ncols();
  

  	if (Qm!=n ){
		cout<<"Wrong matrix dimention"<<endl;
		throw std::logic_error(msg.str());
	}
	else{
		vv.resize(n);		
		for(j=0; j<Qn; j++){
			vv[j]=0;
			for(k=0; k<n; k++){
			vv[j]+=Q[j][k]*x[k];
			}			
		}		
		
	}
  
  return vv;			
}

quadprogpp::Vector<double> VtM(quadprogpp::Vector<double>& x,  quadprogpp::Matrix<double> &Q)
{
  quadprogpp::Vector<double> vv;
  register int  j, k, n = x.size();
  register unsigned int Qn = Q.nrows();
  register unsigned int Qm = Q.ncols();
  

  	if (Qn!=n ){
		cout<<"Wrong matrix dimention"<<endl;
		throw std::logic_error(msg.str());
	}
	else{
		vv.resize(n);		
		for(j=0; j<Qm; j++){
			vv[j]=0;
			for(k=0; k<n; k++){
			vv[j]+=x[k]*Q[k][j];
			}			
		}		
		
	}
  
  return vv;			
}