import numpy as np
import matplotlib.pyplot as plt
import random
alpha = 0.0 #angular acceleration provided by tvc
orientation = 0.0 #the angle wrt normal of earth's surface
omega = 0.0 #angular velocity of rocket
errors = np.empty(1000) #to store deviation from desired angle
def command(angle): #gives command to rotate rocket to certian angle
    global alpha
    alpha = 2*(angle-omega) #alpha to be generated such that the desired angle is reached in 1 second
    if(alpha > 0.4): #however there is a maximum limit on alpha to prevent damage to engine
        alpha=0.4
    if(alpha < -0.4):
        alpha = -0.4


def pid(e,ie,de,p,i,d): #PID condtroller with parameters p,i,d and inputs error, integration error, differentiation error
    p = p
    i = i
    d = d
    command(p*e+i*ie+d*de) #gives command accordingly

def simulation(angle,prop,inte,dee): #simulates rocket dynamics for reaching a desired angle with passed parameters as values of p,i,d
    global alpha, orientation, omega
    alpha = 0.0
    orientation = 0.0
    omega = 0.0
    command(angle)
    ie=0
    de = 0
    fl=0
    eprev=0
    global errors
    f2 = 0
    time = 0.0

    for i in  range(1000): #simulation for 10 seconds
        t = i/100 #since the refresh rate of the IMU sensors is 100Hz
        e = angle-orientation
        if(fl==0):
           eprev = e
           fl = 1

        omega = omega + alpha*(0.01)
        orientation = orientation + omega*(0.01)
        ie = ie + e
        errors[i] = e

        if(i%5==0 and i!=0): #update pid every 0.05 seconds
            de = (e-eprev)/0.05
            fl=0
            pid(e,ie,de,prop,inte,dee)
            ie=0
            if(np.abs(de)< 0.01 and np.abs(e)<0.01 and f2==0): #calculate time required for new orientation to stabilize
                time = t
                f2 = 1
    return time

time = simulation(0.4,0.5,0.2,0.1)
print(time)

#print(errors)
t = np.arange(0.0,10.0,0.01)
plt.plot(t,errors,lw=2)
plt.show() #sample plot shown for specific test values
x = np.array([0.5,0.2,0.1])
def gradient(x,r): #gradient function
    del1 = (simulation(r,x[0]+0.01,x[1],x[2])-simulation(r,x[0],x[1],x[2]))/0.01
    del2 = (simulation(r, x[0] , x[1]+0.01, x[2]) - simulation(r, x[0], x[1], x[2])) / 0.01
    del3 = (simulation(r, x[0] , x[1], x[2]+0.01) - simulation(r, x[0], x[1], x[2])) / 0.01
    grad = np.array([del1,del2,del3])
    return grad

def descent(x,r): #gradient descent
    for i in range(100):

        xnew = x - 0.1*gradient(x,r)

        time1 = simulation(r,x[0],x[1],x[2])
        time2 = simulation(r,xnew[0],xnew[1],xnew[2])

        x = xnew


    return x



final_param_list = np.empty([20,3])

for i in range(20):
    r = random.uniform(-1.0,+1.0)
    final_param = descent(x,r)

    final_param_list[i] = final_param
#optimized values of p,i,d by gradient descent
print(np.mean(final_param_list[0]))
print(np.mean(final_param_list[1]))
print(np.mean(final_param_list[2]))
































































