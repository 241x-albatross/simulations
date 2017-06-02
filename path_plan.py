import numpy as np
import itertools
import matplotlib.pyplot as plt

def loss(mission, my_pos, perm):
    '''
    loss function is alpha*distance + (1- alpha)*turning_angle
    '''
    alpha = 0.5
    
    n,m = mission.shape
    dist = 0.0
    for i in range(n):
        x_1,y_1 = mission[perm[i],:]
        x_0,y_0 = mission[perm[i-1],:] if i>0 else (my_pos[0],my_pos[1])
        dist += np.sqrt((x_1 - x_0)**2 + (y_1 - y_0)**2)




    angle = 0.0
    extend_pos = np.zeros([n+2,2])
    extend_pos[0,:] = my_pos[0] - my_pos[2], my_pos[1] - my_pos[3]
    extend_pos[1,:] = my_pos[0], my_pos[1]
    for i in range(n):
        extend_pos[i+2,:] = mission[perm[i],:]


    for i in range(n):
        x_2,y_2 = extend_pos[i+2,:]
        x_1,y_1 = extend_pos[i+1,:]
        x_0,y_0 = extend_pos[i,:]
        angle += np.arccos(((x_2 - x_1)*(x_1 - x_0) + (y_2 - y_1)*(y_1 - y_0))/(np.sqrt((x_2 - x_1)*(x_2 - x_1) + (y_2 -y_1)*(y_2 - y_1))
                                                                                *np.sqrt((x_1 - x_0)*(x_1 - x_0) + (y_1 -y_0)*(y_1 - y_0))))
       
    return dist*alpha + angle*(1 - alpha)

        
        
        
        
        
    
def plan(mission, x):
    '''
    mission : list of <= 5 mission points,shape is n by 2,
    (x1,y1),(x2,y2)...(xn,yn)
    x current plane state (x0,y0,vx,vy)
    here theta is the heading
    '''
    n,m = mission.shape
    assert(m == 2)
    perm = list(itertools.permutations(np.arange(n), n))
    loss_min = np.inf
    for path in perm:
       
        l = loss(mission, x, path)
        if(l < loss_min):
            opt_path = path
            loss_min = l
    return opt_path
        
        
    
if __name__ == '__main__':

    n = 5
    mission = np.random.rand(n,2)
    x = np.random.rand(4)


    path_id = plan(mission,x)

    print('opt path is ', path_id)
    path = np.zeros([n+1,2])
    path[0,:] = x[0:2]
    for i in range(1,n+1):
        path[i,:] = mission[path_id[i-1],:]
    plt.figure()
    plt.plot(path[:,0],path[:,1], '-o',label = 'missions')

    plt.plot([x[0]-x[2],x[0]],[x[1] - x[3],x[1]], '-ro', label = 'plane and its heading')
    plt.axis('equal')
    plt.legend()
    plt.title('alpha = 0.5')
    plt.show()

