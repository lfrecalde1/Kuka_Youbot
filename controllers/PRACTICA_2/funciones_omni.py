import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
def conversion(v,c,d,r):
    T=np.array([[1,1,c+d],[1,-1,-c-d],[1,-1,c+d],[1,1,-c-d]])*(1/r)
    tranformacion_ruedas=T@v
    return tranformacion_ruedas[0,0],tranformacion_ruedas[1,0],tranformacion_ruedas[2,0],tranformacion_ruedas[3,0]

def tranformacion_cordenadas(x,y,z,phi):
    T=np.matrix([[np.cos(phi),-np.sin(phi),0],[np.sin(phi),np.cos(phi),0],[0,0,1]])
    relativo=np.array([[x],[y],[z]])
    real=T@relativo
    return real[0,0],real[1,0],real[2,0]

def grafica(sty,titulo,x,y,etiqueta,ejex,ejey,color):
    mpl.style.use(sty)
    fig, ax = plt.subplots()
    ax.set_title(titulo.format(sty), color='0')
    ax.set_xlabel(ejex)
    ax.set_ylabel(ejey)
    ax.plot(x, y, color, label=etiqueta)
    ax.grid(linestyle='--', linewidth='0.3', color='black')
    legend = ax.legend(loc='upper right', shadow=False, fontsize='small')
    plt.show()
    
def controlador(h,hd,hdp,q,a,b,k1,k2):
    K1=k1*np.eye(2,2)
    K2=k2*np.eye(2,2)
    herr=hd-h
    J=np.matrix([[np.cos(q[2,0]),-np.sin(q[2,0]),-(a*np.sin(q[2,0])+b*np.cos(q[2,0]))],[np.sin(q[2,0]),np.cos(q[2,0]),a*np.cos(q[2,0])-b*np.sin(q[2,0])]])
    control=np.linalg.pinv(J)@(hdp+K2@np.tanh(np.linalg.inv(K2)@K1@herr))
    return control[0,0], control[1,0], control[2,0]
    

def controlador_secundario(h,hd,hdp,q,qd,a,b,k1,k2,k3,k4):
    K1=k1*np.eye(2,2)
    K2=k2*np.eye(2,2)
    K3=k3*np.eye(3,3)
    K4=k4*np.eye(3,3)
    W=np.eye(3,3)
    
    W_1=np.linalg.inv(W)
    
    herr=hd-h
    
    nulo=qd-q
    
    I=np.eye(3,3)
    
    J=np.matrix([[np.cos(q[2,0]),-np.sin(q[2,0]),-(a*np.sin(q[2,0])+b*np.cos(q[2,0]))],[np.sin(q[2,0]),np.cos(q[2,0]),a*np.cos(q[2,0])-b*np.sin(q[2,0])]])
    
    J_1=np.linalg.pinv(J)
    
    J_m=W_1@J.transpose()@np.linalg.inv(J@W_1@J.transpose())
    
    control=J_m@(hdp+K2@np.tanh(np.linalg.inv(K2)@K1@herr))+(I-J_m@J)@K3@np.tanh(np.linalg.inv(K3)@K4@nulo)
    
    return control[0,0], control[1,0], control[2,0]
    
def grafica_c(sty,titulo,x,y,etiqueta,ejex,ejey,color,x_1,y_1,etiqueta_1,color_1):
    mpl.style.use(sty)
    fig, ax = plt.subplots()
    ax.set_title(titulo.format(sty), color='0')
    ax.set_xlabel(ejex)
    ax.set_ylabel(ejey)
    ax.plot(x, y, color,label=etiqueta)
    ax.plot(x_1,y_1,color_1,label=etiqueta_1)
    ax.plot()
    ax.grid(linestyle='--', linewidth='0.2', color='black')
    legend = ax.legend(loc='upper right', shadow=False, fontsize='small')
    plt.show()