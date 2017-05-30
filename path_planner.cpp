#include <iostream.h>
#include <stdlib.h>
#include <math.h>

using namespace std

//based on distance for now:
double weight(double x2[], double x1[])
{
	double l1=x1[0]-x2[0];
	double l2=x1[1]-x2[1];
	w=sqrt(pow(l1,2)+pow(l2,2));
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
	double new_path[len(path)+1];
	for (int k=0; k<idx; k++){
		new_path[k]=path[k];
		}
	new_path[idx]=pt;
	for (int k=idx+1; k<len(path); k++){
		new_path[k]=path[k-1];
		}
	return new_path;
	}

//input: mission, pos
//output: optimized path 
double DP(double mission[], double pos[])
{
	const int n=len(mission);
	double cost=0;
	double path[]={pos};
	
	for (int k=0; k<n; k++)
	{
		double pt=mission[k];
		double options[k+1];
		for (int i=0; i<k+1; i++){
			double path_opt=insert(path, mission[k], i+1);
			options[i]=0;
			for (int j=0; j<k+2; j++){
				options[i]+=weight(path_opt[j], path_opt[j+1])
				}
		}
		idx=get_lowest(options)
		path=insert(path, mission[k], idx+1)
	}
	return path;
}
