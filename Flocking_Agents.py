import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime
import os
import time


def eucledean(p1, p2):
    #Compute vector r from one agent to the next
    #print(p1,p2)
    r = p1 - p2
    #Compute distance between agents n and m
    rmag = np.sqrt((r[0]**2 + r[1]**2))

    return r, rmag

class store():
    def random_beeline(self, filename, r0=5.7, i_n=2, N=10, frames = 1200, pr = 20, cr = 20):
        #N = 10 #No. of Boids 400
        #frames = 1200 #No. of framesm 100
        limit = 500 #Axis Limits, size of indoor space, one side
        lo_limit = limit * -1
        L  = limit*2
        P = 400 #Spread of initial position (gaussian)
        V = 10 #Spread of initial velocity (gaussian)
        delta = 1 #Time Step
        c1 = 0.1 #Attraction Scaling factor to goal
        c2 = 0.001 #Repulsion scaling factor to all others
        c3 = 1 #Heading scaling factor, not used
        c4 = 0.1 #Randomness scaling factor
        #c5 = 0.01 #Obstacle scaling factor, not used
        c6 = 2.5 # Virus projectile speed
        #r0 = 5.7 #r naught value coronavirus
        #i_n = 2 # number infected to begin
        min_dist_to_goal = 2
        vlimit = 1 #Maximum velocity
        VERBOSE = False
        WALL = True
        PLOT = False
        wall_d = limit
        lo_wall_d = 0
        num_pa = int(2 * r0)#number of passive agents per host
        #pr = 20 #projectile range of virus (in timesteps)
        #cr = 20 #cough rate (in time steps)
        c_zone = 1 #contageous_zone

        TEST = 0
        TEST_N = 10


        #Initialize
        p = P*np.random.random((2,N)) #(x,y)
        v = V*np.random.random((2,N)) #(x,y)

        #Infected boolean array
        ilist = np.zeros(N)
        for n in np.random.randint(0,N,i_n):
            ilist[n] = 1

        #Contageous passive agent array (per agent)
        pas = list()
        for n in range(N):
            pas_xtemp = np.repeat(p[0,n],num_pa,axis=0)
            pas_ytemp = np.repeat(p[1,n],num_pa,axis=0)
            pas_temp = np.stack((pas_xtemp,pas_ytemp), axis = 0)
            pas.append(pas_temp)
            
        # passive agent position
        pas = np.stack(pas, axis = 0)
        #passive agent velocity
        pas_v = np.zeros((N,2,num_pa))
        #passive agent angle
        pas_a = np.zeros((N,2,num_pa))
        for n in range(N):
            tot_rad = 2*math.pi
            rad = tot_rad/num_pa
            for j in range(1,num_pa):
                pas_a[n,:,j] = np.array([float(math.cos(j*rad)), float(math.sin(j*rad))])

        #print (p)
        #print(pas.shape)
        #print(pas_a)
        #quit(0)
        
        #Initializing plot
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111)

        #Obstacle location
        o = np.array([40,0]) #obstacle

        #Goal location
        g = wall_d * np.random.random((2,N))

        #Cough timer
        timer = np.zeros(N)

        with open(filename, "a+") as wFile:
            wFile.write("Timestep,Infected,Infected List\n")
            for i in range(0, frames):
                v1 = np.zeros((2,N)) #attraction
                v2 = np.zeros((2,N)) #repulsion
                        
                v3 = np.mean(v, axis=1) * c3
                if (np.linalg.norm(v3) > vlimit): #limit maximum velocity
                    v3 = v3*vlimit/np.linalg.norm(v3)

                for n in range(0, N):
                    for m in range(0, N):
                        if m!=n:
                            r, rmag = eucledean(p[:,m], p[:,n])

                            if VERBOSE: 
                                print("v1n:%s\n"%(v1[:,n]))
                            #Compute Repulsion [non-linear scaling] v2
                            v2[:,n] -= (c2*r)/(rmag**2)

                            if VERBOSE: 
                                print("v2m:%s\n"%(v2[:,n]))
                                print('done')

                            for j in range(num_pa):
                                #Contage unlucky uninfected active agent that ventured too close
                                r, rmag = eucledean(pas[n,:,j], p[:,m]) #constant
                                if (rmag <= c_zone):
                                    ilist[m] = 1

                        #if (m > 1):
                            #VERBOSE = False

                    #Compute vector from pos agent p to pos obstable o
                    # @todo: #only y position should matter
                    # or, distance from closest wall. 
                    
                    #Compute attraction of current bird n, towards its goal point
                    r, rmag = eucledean(g[:,n], p[:,n]) #constant
                    v1[:,n] += c1*r

                    #re-update goal point if reached
                    if (rmag < min_dist_to_goal):
                        g[:,n] = wall_d * np.random.random(2)

                    #Compute random velocity component v4
                    v4 = c4 * np.random.randn(1,2)
                    if VERBOSE:
                        print("v4:%s\n"%(v4))
                                
                    #Update velocity
                # v[:,n] = v1[:,n] + v2[:,n] + v3 + v4
                    v[:,n] = v1[:,n] + v2[:,n] + v4
                    if VERBOSE:
                        print("vn:%s\n"%(v[:,n]))

                    if WALL:
                        #Compute Static Repulsion Object (wall)
                        #horizontal
                        if (p[1, n] > wall_d):
                            v[1:,n] = v[1:,n] * (-1)
                        if (p[1, n] < lo_wall_d):
                            v[1:,n] = v[1:,n] * (-1)
                        #vertical
                        if (p[0, n] > wall_d):
                            v[0:,n] = v[0:,n] * (-1)
                        if (p[0, n] < lo_wall_d):
                            v[0:,n] = v[0:,n] * (-1)


                    if TEST:
                        if(timer[n] != 0):
                            TEST_N -= 1
                            print("loc_beg", pas[n,:])
                            print("agent", p[:,n])
                    
                    #Contageous passive agent update
                    for j in range(num_pa):  #update all virus every time
                        if (timer[n] != 0): #if sneezing
                            pas[n,:,j] += c6 * pas_a[n,:,j] #virus projectile
                        else:
                            pas[n,:,j] = p[:,n] #virus follows agent

                    if TEST:
                        if(timer[n] != 0):
                            print("vel:", pas_a[n,:])
                            print("loc",pas[n,:])
                            if (TEST_N == 0):
                                quit(1)

                    #Random agent cough
                    if (ilist[n] == 1): #if agent infected
                        if (timer[n] > 0): #if vector sneezing
                            timer[n] -= 1
                        else:
                            #Flip a coin to cough
                            flip = np.random.randint(0,cr) #one in 50 chance
                            if (flip == 1):
                                #Cough!
                                timer[n] = cr #projectile length

                    
                #Update position
                p +=( v * delta)

                if VERBOSE:
                    print("p:%s\n"%(p))

                #Periodic boundary
                tmp_p = p

                tmp_p[0, p[0,:]>L/2] = tmp_p[0,p[0,:]> (L/2)] - L
                tmp_p[1, p[1,:] > L/2] = tmp_p[1, p[1,:] > (L/2)] - L
                tmp_p[0, p[0,:] < -L/2]  = tmp_p[0, p[0,:] < (-L/2)] + L
                tmp_p[1, p[1,:] < -L/2]  = tmp_p[1, p[1,:] < (-L/2)] + L

                p = tmp_p
                # Can Also be written as:
                #p[p > limit] -= limit * 2
                #p[p < -limit] += limit * 2

                # Write to file
                infectList = [n for n, m in enumerate(ilist) if m == 1]
                infected = len(infectList)
                wFile.write("%s,%s,%s\n"%(i,infected,infectList))

                
                if PLOT:
                    line1, = ax.plot(p[0, 0], p[1, 0])
                    #plt.pause(1)


                    #update plot
                    ax.clear()
                    if WALL:
                        #horizontal
                        line2, = ax.plot(np.linspace(lo_wall_d,wall_d, num = wall_d), np.full(wall_d,wall_d))
                        line3, = ax.plot(np.linspace(lo_wall_d,wall_d, num = wall_d), np.full(wall_d,lo_wall_d))
                        #vertical
                        line4, = ax.plot(np.full(wall_d,wall_d), np.linspace(lo_wall_d,wall_d, num = wall_d))
                        line5, = ax.plot(np.full(wall_d,lo_wall_d), np.linspace(lo_wall_d,wall_d, num = wall_d))

                    #goal plot
                    goals, = ax.plot(g[0], g[1], "bo")

                    #infected
                    cmap = np.zeros(N, dtype= object)
                    for n in range(len(ilist)):
                        if (ilist[n] == 0):
                            cmap[n] = (0.7,0.7,0.7,0.5)
                        else:
                            cmap[n] = (0.6,0.,0.,0.7)
                    
                    #passive agents plot
                    #pas_flat = pas.reshape(N*num_pa,2)
                    for n in range(N):
                        if (ilist[n] == 1):
                            ax.plot(pas[n,0,:], pas[n,1,:], "ro")

                    #obstacles plot
                    #obstacle, = ax.plot(o[0],o[1], "ro")
                    ax.quiver(p[0,:], p[1,:], v[0,:], v[1,:], color = cmap) # For drawing velocity arrows
                    #plt.pause(1) #thrashing axes
                    plt.xlim(0, limit)
                    plt.ylim(0, limit)
                    line1.set_data(p[0,:], p[1,:])

                    plt.pause(0.0001)
                    fig.canvas.draw()

        if PLOT:        
            plt.show(block=True)

    




beeline_py = store()

r0=5.7
i_n=1
N=10
frames = 2460 #half an hour
#r0_list=
#N_list=
#in_list=

#Make run folder
folder = "data\\test_%s"%(int(datetime.datetime.now().timestamp()))
os.mkdir(folder)

#beeline_py.random_beeline("test01",r0, i_n, N, frames)

"""
#R_0 value
for r0 in range(1,16,1):
    p = datetime.datetime.now().timestamp()
    r0 = r0*0.5
    filename = "%s\\flockingagents_r0=%s_in=%s_N=%s_frames=%s"%(folder,r0, i_n, N, frames)
    print ("Working on %s/16.0: %s\n"%(r0, filename))
    beeline_py.random_beeline(filename,r0, i_n, N, frames)
    q = datetime.datetime.now().timestamp()
    print( "Finished in %s seconds\n"%(q-p))

"""

"""
r0=5.7
i_n=1
N=10
frames = 2460 #half an hour

# Starting infected value
for i_n in range(1,10):
    p = datetime.datetime.now().timestamp()
    filename = "%s\\flockingagents_r0=%s_in=%s_N=%s_frames=%s"%(folder,r0, i_n, N, frames)
    print ("Working on : %s\n"%(filename))
    beeline_py.random_beeline(filename,r0, i_n, N, frames)
    q = datetime.datetime.now().timestamp()
    print( "Finished in %s seconds\n"%(q-p))
"""

"""
r0=5.7
i_n=1
N=10
frames = 2460 #half an hour

# number of users
for N in range(20, 30):
    p = datetime.datetime.now().timestamp()
    filename = "%s\\flockingagents_r0=%s_in=%s_N=%s_frames=%s"%(folder,r0, i_n, N, frames)
    print ("Working on : %s\n"%(filename))
    beeline_py.random_beeline(filename,r0, i_n, N, frames)
    q = datetime.datetime.now().timestamp()
    print( "Finished in %s seconds\n"%(q-p))
"""    

r0=5.7
i_n=1
N=10
frames = 3600 #hour

# amount of time in store
for r0 in [2,3,5.7]:
    for frames in range(300,3600,100): #up to an hour
        p = datetime.datetime.now().timestamp()
        filename = "%s\\flockingagents_r0=%s_in=%s_N=%s_frames=%s"%(folder,r0, i_n, N, frames)
        print ("Working on : %s\n"%(filename))
        beeline_py.random_beeline(filename,r0, i_n, N, frames)
        q = datetime.datetime.now().timestamp()
        print( "Finished in %s seconds\n"%(q-p))


