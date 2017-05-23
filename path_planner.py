import numpy as np
import matplotlib.pyplot as plt
import pdb


#----------------DP
# input: mission as a list of arrays
#        x - state of plane (array)
# output: mission_sorted
# variables: W matrix contrinaing all tge wieght for the DP alorithms


def DP_sort(mission, x):

	n=len(mission)
	W=np.zeros((n,n+1))
	path=[0] #structure of path i is [mission points, cost]
	cost=0.
	for i in range(n):
		W[i,0]=np.linalg.norm(x-mission[i])
		for j in range(n):
			W[i,j+1]=np.linalg.norm(mission[i]-mission[j])

	for i in range(n):
		i+=1 # cause 0 is starting point
		opt=np.ones(i)
		opt=opt*cost
		for j in range(i):
			opt[j]+=W[i-1,path[j]]

		cost=min(opt)
		path.insert(np.argmin(opt)+1, i)

	return path

def DP_dyn_sort(mission, x):

	n=len(mission)
	path=[x[:2]] #structure of path i is [mission points, cost]
	cost=0.

	for i in range(n):
		i+=1 # cause 0 is starting point
		opt=np.ones(i)
		opt=opt*cost
		for j in range(i):
			path_opt=path.insert(j+1, mission[i])
			controller=PathController(path_opt)
			stats=rollout(controller.policy, x, 100, .1)
			pos=np.vstack(stats["x"])
			x=pos[:,0]
			y=pos[:,1]
			z=pos[:,2]
			w=0.
			for i in range(len(x)-1): 
				w+=np.sqrt((x[i]-x[i+1])**2.+(y[i]-y[i+1])**2.+(z[i]-z[i+1])**2.)
			opt[j]+=w

		cost=min(opt)
		path.insert(np.argmin(opt)+1, mission[i])

	return path


#----------------------------------
# input: mission as it was (only pts ahead, not the passed ones!), x is current state, new point new point
# output: path -list of new points

#----------------------------------

def DP_add_pt(mission, x, new_pts):
	
	m=len(mission)
	n=len(new_pts)
	W=np.zeros((m,n+1+m))
	x=x[:2]
	path=mission #structure of path i is [mission points, cost]
	cost=0.

	for i in range(n):
		W[i,0]=np.linalg.norm(x-new_pts[i])
		for j in range(m):
			W[i,j+1]=np.linalg.norm(new_pts[i]-mission[j])
		for k in range(n):
			W[i,m+1+k]=np.linalg.norm(new_pts[i]-new_pts[k])

	for i in range(n):
		i+=m+1
		opt=np.ones(i)
		opt=opt*cost
		for j in range(i):
			opt[j]+=W[i-(n+1),j]

		cost=min(opt)
		path.insert(np.argmin(opt)+1, new_pts[i-(n+1)])

	return path

	
