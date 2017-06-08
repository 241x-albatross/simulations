import numpy as np
import matplotlib.pyplot as plt
import pdb
import copy

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

	opt=[x]
	for i in path[1:]:
		opt.append(mission[i-1])

	return opt

def weight(cur, next,bef, p=10):
	w_d=np.linalg.norm(cur-next)
	
	w_a=0.
	v1=next.T-cur.T
	v2=bef.T-cur.T
	angel=(180.0/np.pi)*np.arccos( (v1[0]*v2[0]+v1[1]*v2[1]) / (np.linalg.norm(v1)*np.linalg.norm(v2)) )
	print(angel)
	if angel<100.0:
		w_a+=p;
	elif angel<50.0:
		w_a+=p;

	w=w_d+w_a

	return w

def DP(mission, x):

	n=len(mission)
	path=[x]
	cost=0.

	for i in range(n):
		opt=np.zeros(i+1)
		for j in range(i+1):
			path_opt=copy.copy(path)
			path_opt.insert(j+1, mission[i])

			opt[j]=np.linalg.norm(path_opt[0]-path_opt[1])
			if i>2:
				for j in range(i-1):
					opt[i]+=weight(path_opt[j+1], path_opt[j+2], path_opt[j])

			del path_opt


		cost=min(opt)
		path.insert(np.argmin(opt)+1, mission[i-1])

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

	


if __name__  == "__main__":

	x=np.array([20., 100.])
	N=[200., 0., 100., -100., 0.];
	E=[0., 0., -50., -100., -35.];
	mission=[]
	for i in range(len(N)):
		mission.append(np.array([N[i], E[i]]))

	path=DP(mission, x[0:2])
	print path
	M=np.array(path)

	plt.plot(M[:,1], M[:,0], 'b*', M[:,1], M[:,0], 'r-')
	plt.grid(True)
	plt.xlabel('Eastern Position [m]')
	plt.ylabel('Nothern Position [m]')
	plt.axis([-150, 100, -150, 250])
	plt.show()

