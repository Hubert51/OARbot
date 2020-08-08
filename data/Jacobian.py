n[8]:


import numpy as np
from math import pi
import sympy as sym


# In[15]:


#T1 = sym.Symbol('T1')
#T2 = sym.Symbol('T2')
#T3 = sym.Symbol('T3')
##T4 = sym.Symbol('T4')
##T5 = sym.Symbol('T5')
#T6 = sym.Symbol('T6')

T1 = 0
T2 = 0
T3 = 0
T4 = 0
T5 = 0
T6 = 0


T1 = (T1/180.0)*pi
T2 = (T2/180.0)*pi
T3 = (T3/180.0)*pi
T4 = (T4/180.0)*pi
T5 = (T5/180.0)*pi
T6 = (T6/180.0)*pi

L1 = 0.2755
L2 = 0.4100
L3 = 0.2073
L4 = 0.0741
L5 = 0.0741
L6 = 0.1600

PT = [[T1, (90.0/180.0)*pi, 0, L1],
      [T2, pi, L2, 0],
      [T3+(90.0/180.0)*pi, (-90.0/180.0)*pi, 0, 0],
      [T4, 0, 0, -L3-L4],
      [T5+(90.0/180.0)*pi, pi, 0 -L5],
      [T6, 0, 0, L6]]


# In[16]:


i = 0
H0_1 = np.array([[np.cos(PT[i][0]), -np.sin(PT[i][0])*np.cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]), np.cos(PT[i][0])*np.cos(PT[i][1]), -np.cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
        [0, np.sin(PT[i][1]), np.cos(PT[i][1]), PT[1][3]],
        [0, 0, 0, 1]])

i = 1
H1_2 = np.array([[np.cos(PT[i][0]), -np.sin(PT[i][0])*np.cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]), np.cos(PT[i][0])*np.cos(PT[i][1]), -np.cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
        [0, np.sin(PT[i][1]), np.cos(PT[i][1]), PT[1][3]],
        [0, 0, 0, 1]])

i = 2
H2_3 = np.array([[np.cos(PT[i][0]), -np.sin(PT[i][0])*np.cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]), np.cos(PT[i][0])*np.cos(PT[i][1]), -np.cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
        [0, np.sin(PT[i][1]), np.cos(PT[i][1]), PT[1][3]],
        [0, 0, 0, 1]])

i = 3
H3_4 = np.array([[np.cos(PT[i][0]), -np.sin(PT[i][0])*np.cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]), np.cos(PT[i][0])*np.cos(PT[i][1]), -np.cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
        [0, np.sin(PT[i][1]), np.cos(PT[i][1]), PT[1][3]],
        [0, 0, 0, 1]])

i = 4
H4_5 = np.array([[np.cos(PT[i][0]), -np.sin(PT[i][0])*np.cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]), np.cos(PT[i][0])*np.cos(PT[i][1]), -np.cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
        [0, np.sin(PT[i][1]), np.cos(PT[i][1]), PT[1][3]],
        [0, 0, 0, 1]])


I = 5
H5_6 = np.array([[np.cos(PT[i][0]), -np.sin(PT[i][0])*np.cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]), np.cos(PT[i][0])*np.cos(PT[i][1]), -np.cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
        [0, np.sin(PT[i][1]), np.cos(PT[i][1]), PT[1][3]],
        [0, 0, 0, 1]])


# In[17]:


print(H0_1)


# In[18]:


H0_2 = np.dot(H0_1,H1_2)
H0_3 = np.dot(H0_2, H2_3)
H0_4 = np.dot(H0_3,H3_4)
H0_5 = np.dot(H0_4,H4_5)
H0_6 = np.dot(H0_5,H5_6)


# In[19]:


test = np.arange(35).reshape(5,7)


# In[20]:


print(test)


# In[21]:


print(test[...,6][0:4])


# In[22]:


D0_1 = H0_1[...,3][0:3]
D0_2 = H0_2[...,3][0:3]
D0_3 = H0_3[...,3][0:3]
D0_4 = H0_4[...,3][0:3]
D0_5 = H0_5[...,3][0:3]
D0_6 = H0_6[...,3][0:3]


# In[23]:


R0_0 = np.array([0,0,1])
R0_1 = H0_1[...,2][0:3]
R0_2 = H0_2[...,2][0:3]
R0_3 = H0_3[...,2][0:3]
R0_4 = H0_4[...,2][0:3]
R0_5 = H0_5[...,2][0:3]
R0_6 = H0_6[...,2][0:3]

test1 = np.cross(R0_0, D0_6)
test2 = np.cross(R0_1, (D0_6-D0_1))
test3 = np.cross(R0_2, (D0_6-D0_2))
test4 = np.cross(R0_3, (D0_6-D0_3))
test5 = np.cross(R0_4, (D0_6-D0_4))
test6 = np.cross(R0_5, (D0_6-D0_5))


# In[26]:


Jacobian = np.array([[test1[0], test2[0], test3[0], test4[0], test5[0], test6[0]],
                     [test1[1], test2[1], test3[1], test4[1], test5[1], test6[1]],
                     [test1[2], test2[2], test3[2], test4[2], test5[2], test6[2]],
                     [R0_0[0], R0_1[0], R0_2[0], R0_3[0], R0_4[0], R0_5[0]],
                     [R0_0[1], R0_1[1], R0_2[1], R0_3[1], R0_4[1], R0_5[1]],
                     [R0_0[2], R0_1[2], R0_2[2], R0_3[2], R0_4[2], R0_5[2]]])


# In[27]:


print(Jacobian)


# In[ ]:

