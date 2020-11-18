"""Practica_2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import *
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
import time
import matplotlib as mpl
from funciones_omni import *

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(100)# 100 milisegundos equivale a 0.1 segundos

## Parte de declaracion de los motores del robot
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']

for i in range(4):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

# DEFINCION DEL SENSOR GPS
gps = GPS("gps")
gps.enable(timestep)

# DFINCION DE LA IMU
imu = InertialUnit("inertial unit")
imu.enable(timestep)

# Definir el tiempo de sampleo del sistema
t_sample=0.1
t_final=30+t_sample
t=np.arange(0,t_final,t_sample)
t=t.reshape(1,t.shape[0])

#Parametros del robot omnidirecional
a=0.1510                         # se saco de las dimensiones reales del robot
b=0.15                            # se como no se mueve en b es cero
c=0.2455                         # se saco de las dimensiones reales
d=0.2080                         # se saco de las diensiones reales
r=0.05                          # radio de las ruedas

#velocidades generales
ul=1*np.ones((t.shape[0],t.shape[1]))
um=0*np.ones((t.shape[0],t.shape[1]))
w=0*np.ones((t.shape[0],t.shape[1]))

#velocidades de cada rueda
w_1=0*np.ones((t.shape[0],t.shape[1]))
w_2=0*np.ones((t.shape[0],t.shape[1]))
w_3=0*np.ones((t.shape[0],t.shape[1]))
w_4=0*np.ones((t.shape[0],t.shape[1]))

#Posiciones del robot
x=np.zeros((t.shape[0],t.shape[1]+1))
y=np.zeros((t.shape[0],t.shape[1]+1))
phi=np.zeros((t.shape[0],t.shape[1]+1))

if robot.step(timestep) != -1:
    #posiciones iniciales lectura del robot
    posicion = gps.getValues()
    #Tranformacion de las posiciones reales al sistema de referencia deseado
    x_real,y_real,z_real=tranformacion_cordenadas(posicion[2],posicion[0],posicion[1],-np.pi/2)
    phi[0,0]=imu.getRollPitchYaw()[2]
    x[0,0]=x_real+a*np.cos(phi[0,0])-b*np.sin(phi[0,0])
    y[0,0]=y_real+a*np.sin(phi[0,0])+b*np.cos(phi[0,0])
    
# Trayectoria Deseada
xd=1*np.cos(0.2*t)
yd=1*np.sin(0.2*t)
phi_d=(0*np.pi)/180*np.ones((t.shape[0],t.shape[1]))

#trayectoria deseada derivada
xd_p=-1*0.2*np.sin(0.2*t)
yd_p= 0.2*1*np.cos(0.2*t)

#ganancias del controlador
k1=1
k2=0.2
k3=1
k4=1

# Errores de control
herrx=np.zeros((t.shape[0],t.shape[1]))
herry=np.zeros((t.shape[0],t.shape[1]))
qerr1=np.zeros((t.shape[0],t.shape[1]))

# Main loop:
for k in range(0,t.shape[1]):
    if robot.step(timestep) != -1:
    
        # Seccion para almacenar los valores de los errors del sistema
        herrx[0,k]=xd[0,k]-x[0,k]
        herry[0,k]=yd[0,k]-y[0,k]
        qerr1[0,k]=phi_d[0,k]-phi[0,k]
        # Definicion del vector de posiciones
        h=np.array([[x[0,k]],[y[0,k]]])

        #Definicion del vector de posiciones deseadas
        hd=np.array([[xd[0,k]],[yd[0,k]]])

        # Definicion de la derivada de la posicion deseada
        hdp=np.array([[xd_p[0,k]],[yd_p[0,k]]])
    
        # definicion del vector de controlador secundario
        q=np.array([[0],[0],[phi[0,k]]])
    
        #deficnion de los deseados del controlador secundario
        qd=np.array([[0],[0],[phi_d[0,k]]])
    
        #controlador basado en Lyapunov
        ul[0,k],um[0,k],w[0,k]=controlador(h,hd,hdp,q,a,b,k1,k2)
 
        #controlador basado en Lyapunov
        #ul[0,k],um[0,k],w[0,k]=controlador_secundario(h,hd,hdp,q,qd,a,b,k1,k2,k3,k4)
        # Vector de velocidades del sistema
        v=np.array([[ul[0,k]],[um[0,k]],[w[0,k]]])

        # Conversion a las velociades de las ruedas
        w_1[0,k],w_2[0,k],w_3[0,k],w_4[0,k]=conversion(v,c,d,r)

        # Envio de comandos a las ruedas
        wheels[0].setVelocity(w_1[0,k])
        wheels[1].setVelocity(w_2[0,k])
        wheels[2].setVelocity(w_3[0,k])
        wheels[3].setVelocity(w_4[0,k])

        # Obtencion de los datos del gps
        posicion = gps.getValues()
        
        x_real,y_real,z_real=tranformacion_cordenadas(posicion[2],posicion[0],posicion[1],-np.pi/2)

        phi[0,k+1]=imu.getRollPitchYaw()[2]

        x[0,k+1]=x_real+a*np.cos(phi[0,k+1])-b*np.sin(phi[0,k+1])
        y[0,k+1]=y_real+a*np.sin(phi[0,k+1])+b*np.cos(phi[0,k+1])
        

        print(x[0,k+1],y[0,k+1],phi[0,k+1])

wheels[0].setVelocity(0)
wheels[1].setVelocity(0)
wheels[2].setVelocity(0)
wheels[3].setVelocity(0)

grafica_c('default','Trayectoria',x[0,:],y[0,:],'$\mathbf{\eta(t)}$','$x[m]$','$y[m]$','b',xd[0,:],yd[0,:],'$\mathbf{\eta_{d}(t)}$','g')
grafica_c('default','Trayectoria',t[0,:],herrx[0,:],'$E_x$','$t[s]$','$E$','b',t[0,:],herry[0,:],'$E_y$','g')
grafica('default','Orientacion',t[0,:],phi[0,0:(phi.shape[1]-1)],'$P_{hi}$','$t[s]$','$phi$','g')
grafica('default','Errores de Orientacion',t[0,:],qerr1[0,:],'$E_{phi}$','$t[s]$','$E$','g')
grafica('default','Velocidad Frontal',t[0,:],ul[0,:],'$\mu_l(t)$','$t[s]$','$[m/s]$','g')
grafica('default','Velocidad Lateral',t[0,:],um[0,:],'$\mu_m(t)$','$t[s]$','$[m/s]$','b')
grafica('default','Velocidad Angular',t[0,:],w[0,:],'$\omega(t)$','$t[s]$','$[rad/s]$','r')
grafica('default','Velocidad angular 1',t[0,:],w_1[0,:],'$\omega_{1}(t)$','$t[s]$','$[rad/s]$','r')
grafica('default','Velocidad angular 2',t[0,:],w_2[0,:],'$\omega_{2}(t)$','$t[s]$','$[rad/s]$','b')
grafica('default','Velocidad angular 3',t[0,:],w_3[0,:],'$\omega_{3}(t)$','$t[s]$','$[rad/s]$','g')
grafica('default','Velocidad angular 4',t[0,:],w_4[0,:],'$\omega_{4}(t)$','$t[s]$','$[rad/s]$','m')
print("FINALIZACION DEL PROGRAMA")
# Enter here exit cleanup code.


