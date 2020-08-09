#!/usr/bin/env python
# coding: utf-8

# In[80]:


"""
computation of jacobian matrix for the Kinova Jaco 6 DOF curved wrist using official DH parameter. The offical 
specification manual can be found here:
https://www.kinovarobotics.com/sites/default/files/ULWS-RA-JAC-UG-INT-EN%20201804-1.0%20%28KINOVA%E2%84%A2%20Ultra%20lightweight%20robotic%20arm%20user%20guide%29_0.pdf
"""
import numpy as np
from math import pi
import math
import sympy as sym


# In[76]:


#define angle in each frame
T1 = sym.symbols('T1')
T2 = sym.symbols('T2')
T3 = sym.symbols('T3')
T4 = sym.symbols('T4')
T5 = sym.symbols('T5')
T6 = sym.symbols('T6')


# In[55]:


#specification of jaco
D1 = 0.2755
D2 = 0.4100
D3 = 0.2073
D4 = 0.0741
D5 = 0.0741
D6 = 0.1600
e2 = 0.0098
aa = (30.0*pi)/180.0
sa = sym.sin(aa)
s2a = sym.sin(2*aa)
d4d = D3+(sa/s2a)*D4
d5d = (sa/s2a)*D4 + (sa/s2a)*D5
d6d = (sa/s2a)*D5+D6


# In[62]:


#D-H parameter table. ordered in theta,alpha, r and d
PT = np.array([[(-T1), (90.0/180.0)*pi, 0, D1],
      [T2+(90.0/180.0)*pi, pi, D2, 0],
      [T3-(90.0/180.0)*pi, (90.0/180.0)*pi, 0, -e2],
      [T4, 2*aa, 0, -d4d],
      [T5+pi, 2*aa, 0 -d5d],
      [T6-(90.0/180.0)*pi, pi, 0, -d6d]])
#j = 0
#print(sym.cos(PT[j][1]))


# In[67]:


i = 0
H0_1 = np.array([[sym.cos(PT[i][0]), -sym.sin(PT[i][0])*sym.cos(PT[i][1]), sym.sin(PT[i][0])*sym.sin(PT[i][1]), PT[i][2]*sym.cos(PT[i][0])],
        [sym.sin(PT[i][0]), sym.cos(PT[i][0])*sym.cos(PT[i][1]), -sym.cos(PT[i][0])*sym.sin(PT[i][1]), PT[i][2]*sym.sin(PT[i][0])],
        [0, sym.sin(PT[i][1]), sym.cos(PT[i][1]), PT[1][3]],
        [0, 0, 0, 1]])

i = 1
H1_2 = np.array([[sym.cos(PT[i][0]), -sym.sin(PT[i][0])*sym.cos(PT[i][1]), sym.sin(PT[i][0])*sym.sin(PT[i][1]), PT[i][2]*sym.cos(PT[i][0])],
        [sym.sin(PT[i][0]), sym.cos(PT[i][0])*sym.cos(PT[i][1]), -sym.cos(PT[i][0])*sym.sin(PT[i][1]), PT[i][2]*sym.sin(PT[i][0])],
        [0, sym.sin(PT[i][1]), sym.cos(PT[i][1]), PT[1][3]],
        [0, 0, 0, 1]])

i = 2
H2_3 = np.array([[sym.cos(PT[i][0]), -sym.sin(PT[i][0])*sym.cos(PT[i][1]), sym.sin(PT[i][0])*sym.sin(PT[i][1]), PT[i][2]*sym.cos(PT[i][0])],
        [sym.sin(PT[i][0]), sym.cos(PT[i][0])*sym.cos(PT[i][1]), -sym.cos(PT[i][0])*sym.sin(PT[i][1]), PT[i][2]*sym.sin(PT[i][0])],
        [0, sym.sin(PT[i][1]), sym.cos(PT[i][1]), PT[1][3]],
        [0, 0, 0, 1]])

i = 3
H3_4 = np.array([[sym.cos(PT[i][0]), -sym.sin(PT[i][0])*sym.cos(PT[i][1]), sym.sin(PT[i][0])*sym.sin(PT[i][1]), PT[i][2]*sym.cos(PT[i][0])],
        [sym.sin(PT[i][0]), sym.cos(PT[i][0])*sym.cos(PT[i][1]), -sym.cos(PT[i][0])*sym.sin(PT[i][1]), PT[i][2]*sym.sin(PT[i][0])],
        [0, sym.sin(PT[i][1]), sym.cos(PT[i][1]), PT[1][3]],
        [0, 0, 0, 1]])

i = 4
H4_5 = np.array([[sym.cos(PT[i][0]), -sym.sin(PT[i][0])*sym.cos(PT[i][1]), sym.sin(PT[i][0])*sym.sin(PT[i][1]), PT[i][2]*sym.cos(PT[i][0])],
        [sym.sin(PT[i][0]), sym.cos(PT[i][0])*sym.cos(PT[i][1]), -sym.cos(PT[i][0])*sym.sin(PT[i][1]), PT[i][2]*sym.sin(PT[i][0])],
        [0, sym.sin(PT[i][1]), sym.cos(PT[i][1]), PT[1][3]],
        [0, 0, 0, 1]])


i = 5
H5_6 = np.array([[sym.cos(PT[i][0]), -sym.sin(PT[i][0])*sym.cos(PT[i][1]), sym.sin(PT[i][0])*sym.sin(PT[i][1]), PT[i][2]*sym.cos(PT[i][0])],
        [sym.sin(PT[i][0]), sym.cos(PT[i][0])*sym.cos(PT[i][1]), -sym.cos(PT[i][0])*sym.sin(PT[i][1]), PT[i][2]*sym.sin(PT[i][0])],
        [0, sym.sin(PT[i][1]), sym.cos(PT[i][1]), PT[1][3]],
        [0, 0, 0, 1]])


# In[68]:


H0_2 = np.dot(H0_1,H1_2)
H0_3 = np.dot(H0_2, H2_3)
H0_4 = np.dot(H0_3,H3_4)
H0_5 = np.dot(H0_4,H4_5)
H0_6 = np.dot(H0_5,H5_6)


# In[69]:


D0_1 = H0_1[...,3][0:3]
D0_2 = H0_2[...,3][0:3]
D0_3 = H0_3[...,3][0:3]
D0_4 = H0_4[...,3][0:3]
D0_5 = H0_5[...,3][0:3]
D0_6 = H0_6[...,3][0:3]


# In[70]:


R0_0 = np.array([0,0,1])
R0_1 = H0_1[...,2][0:3]
R0_2 = H0_2[...,2][0:3]
R0_3 = H0_3[...,2][0:3]
R0_4 = H0_4[...,2][0:3]
R0_5 = H0_5[...,2][0:3]
R0_6 = H0_6[...,2][0:3]


# In[71]:


test1 = np.cross(R0_0, D0_6)
test2 = np.cross(R0_1, (D0_6-D0_1))
test3 = np.cross(R0_2, (D0_6-D0_2))
test4 = np.cross(R0_3, (D0_6-D0_3))
test5 = np.cross(R0_4, (D0_6-D0_4))
test6 = np.cross(R0_5, (D0_6-D0_5))


# In[72]:


Jacobian = np.array([[test1[0], test2[0], test3[0], test4[0], test5[0], test6[0]],
                     [test1[1], test2[1], test3[1], test4[1], test5[1], test6[1]],
                     [test1[2], test2[2], test3[2], test4[2], test5[2], test6[2]],
                     [R0_0[0], R0_1[0], R0_2[0], R0_3[0], R0_4[0], R0_5[0]],
                     [R0_0[1], R0_1[1], R0_2[1], R0_3[1], R0_4[1], R0_5[1]],
                     [R0_0[2], R0_1[2], R0_2[2], R0_3[2], R0_4[2], R0_5[2]]])


# In[73]:


print(Jacobian)


# In[ ]:




