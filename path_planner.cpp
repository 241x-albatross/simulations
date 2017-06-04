#include <iostream.h>
#include <stdlib.h>
#include <math.h>

using namespace std

//based on distance + penalty for high angles
double weight(double x1[], double x2[], double v, double x_prev[])
{
	//distance eval
	double l1=x2[0]-x1[0];
	double l2=x2[1]-x1[1];
	double v1[]={l1, l2};
	w_d=sqrt(pow(l1,2)+pow(l2,2));
	
	const double pi= 3.141592653589793238462643383279502884;
	
	//angle
	double v2[]={x_prev[0]-x1[0],x_prev[1]-x1[1]};
	double theta=(180.0/pi)*acos((v1[0]*v2[0]+v1[1]*v2[1]) /(pow(v1[0],2)+pow(v1[1],2))/ (pow(v2[0],2)+pow(v2[1],2)));
	double w_angel=0.0;
	
	//penalty 
	const double p=20.0;
	
	if (w_angel<120.0){
		w_angle+=p;
		};
	if (w_angel<60.0){
		w_angle+=p;
		};
	if (w_angel<30.0){
		w_angle+=pow(10,10);
		};
	
	w=w_d+w_angle
	
	if (x_prev.size()==1) {
		return w_d
		}
	else
		return w;
	}

//find lowest cost among options
int get_lowest(double vec[])
{
	int idx=100;
	double min=1000000;
	for (int i=0; i<vec.size(); i++){
		if (vec[i]<min)
			min=vec[i];
			idx=i;
		}
	return idx;
	}

//inster point pt at position idx in path
double insert(double path[], double pt[], int idx)
{
	const int pl=path.size();
	double new_path[pl+1];
	for (int k=0; k<idx; k++){
		new_path[k]=path[k];
		}
	new_path[idx]=pt;
	for (int k=idx+1; k<pl; k++){
		new_path[k]=path[k-1];
		}
	return new_path;
	}

//input: mission, pos
//output: optimized path 
double DP(double mission[], double pos[])
{
	const int n=mission.size();
	double cost=0;
	double path[]={pos};
	
	//cruise velocity
	const double v=15.5;
	
	for (int k=0; k<n; k++)
	{
		double pt[]=mission[k];
		double options[k+1];
		for (int i=0; i<k+1; i++){
			double path_opt[]=insert(path, mission[k], i+1);
			options[i]=weight(path_opt[0], path_opt[1], v, 1.0);
			for (int j=1; j<k+2; j++){
				options[i]+=weight(path_opt[j], path_opt[j+1], v, path_opt[j-1])
				}
		}
		idx=get_lowest(options)
		path=insert(path, mission[k], idx+1)
	}
	return path;
}
