#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import math
from pyquaternion import Quaternion
import warnings

def AxisAngle(A):

    warnings.filterwarnings("ignore")
    p = np.array(np.linalg.eig(A)[1][:,2],dtype=float)
    warnings.resetwarnings()

    u = np.array([-p[1],p[0],0])
    u = u / np.linalg.norm(u)
    up = np.dot(A,u)
    fi = np.arccos(np.clip(np.dot(u, up), -1.0, 1.0))
    if np.linalg.det(np.array([u,up,p])) < 0:
        p = -p
    return (p,fi)
def A2Euler(A):
    omega = 0
    phi = 0
    theta = 0
    if A[2][0] < 1:
        if A[2][0] > -1:
            omega = np.arctan2(A[1][0],A[0][0])
            theta = np.arcsin(-A[2][0])
            phi = np.arctan2(A[2][1],A[2][2])
        else:
            omega = np.arctan2(-A[0][1],A[1][1])
            theta = math.pi / 2
            phi = 0

    else:
        omega = np.arctan2(-A[0][1],A[1][1])
        theta = -math.pi / 2
        phi = 0
    return (phi,theta,omega)

def Euler2A(phi,theta,omega):

    R_phi = np.array([[1,0,0],
                    [0,np.cos(phi),-np.sin(phi)],
                    [0,np.sin(phi),np.cos(phi)]])

    R_theta = np.array([[np.cos(theta),0,np.sin(theta)],
                        [0,1,0],
                        [-np.sin(theta),0,np.cos(theta)]])

    R_omega = np.array([[np.cos(omega),-np.sin(omega),0],
                        [np.sin(omega),np.cos(omega),0],
                        [0,0,1]])

    return np.dot(np.dot(R_omega,R_theta),R_phi)

def Rodrigez(p,phi):

    p = p / np.linalg.norm(p)
    px = np.array([[0,-p[2],p[1]],
                    [p[2],0,-p[0]],
                    [-p[1],p[0],0]])

    ppt = np.empty((3,3))
    #Transpose se u numpy ne ponasa ocekivano na 1d nizovima pa sam morao rucno da izracunam
    for i in range(0,3):
        for j in range(0,3):
            ppt[i][j] = p[i] * p[j]

    Rpt = np.empty((3,3))
    Rpt = ppt + np.cos(phi)*(np.eye(3)-ppt) + np.sin(phi)*px
    return Rpt

def Q2AxisAngle(q):
    q = Quaternion(q)
    q = q.normalised
    if q.real < 0:
        q = -q
    phi = 2 * np.arccos(q.real)
    if q.real == 1:
        p = np.array([1,0,0])
    else:
        p = q.imaginary
        p = p / np.linalg.norm(p)

    return (p,phi)
def AngleAxis2Q(p,phi):

    w = np.cos(phi / 2)
    p = p / np.linalg.norm(p)
    imaginary = np.sin(phi/2)*p
    q = Quaternion(real=w, imaginary=imaginary)
    return q
def main():

    phi = -np.arctan(np.pi / 2)
    theta = -np.arcsin(np.pi / 4)
    omega =  np.arctan(np.pi / 3)
    print("\u03D5:",phi)
    print("\u03B8: ",theta)
    print("\u03A9: ",omega,"\n")

    print("----Euler2A----")
    A = Euler2A(phi,theta ,omega)
    print("A: ")
    print(A,"\n")
    print("----AxisAngle----")
    p, phi = AxisAngle(A)
    print("p: ",p)
    print("\u03D5:",phi,"\n")
    print("----Rodrigez----")
    A = Rodrigez(p,phi)
    print("A:")
    print(A,"\n")
    print("----AngleAxis----")
    phi1, theta1, omega1 = A2Euler(A)
    print("\u03D5:",phi1)
    print("\u03B8: ",theta1)
    print("\u03A9: ",omega1,"\n")
    print("----AngleAxis2Q---")
    q = AngleAxis2Q(p,phi)
    print("Quaternion: ", q, "\n")
    print("----Q2AxisAngle----")
    p1, phi1 = Q2AxisAngle(q)
    print("p: ",p1)
    print("\u03D5:",phi1,"\n")
    print("Ulaz za narednu funkciju je izlaz iz prethodne, sto se moze videti u kodu")

if __name__ == "__main__":
    main()
