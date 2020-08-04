#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import random as rm
import pandas as pd
import math 
import matplotlib.pyplot as plt


# In[2]:


def ranNode():#generator of nodes
    x=rm.uniform(-0.5,0.5)
    y=rm.uniform(-0.5,0.5)
    return (x,y)
def plotValues():#plot nodes
    xval=pts[:,1];
    yval=pts[:,2];
    for i in range(obs.shape[0]):
        circle1=plt.Circle((obs[i,0],obs[i,1]),(obs[i,2]/2),color='r')
        plt.gcf().gca().add_artist(circle1)
    plt.scatter(xval,yval,marker=".")
    #plt.savefig('plot.png')
    
def validNode(x,y): #check if xy inside obstacle
    for i in range(obs.shape[0]):
        if((x - obs[i,0])**2 + (y - obs[i,1])**2 <=(obs[i,2]/2+0.02)**2):
            return False
    return True
def heuristic(x,y): #calculate cost to goal distance
    h=((x-goal[0])**2+(y-goal[1])**2)**0.5
    return h

def genNode():#node generator
    global pts
    
    c=1;
    pts=np.append(pts,[c,init[0],init[1],init[2]]);
    c=c+1
    for i in range(k):
    #while(pts.shape[0]<(k-1)):
        (x,y)=ranNode();
        if(validNode(x,y)==False):
               continue
        h=heuristic(x,y)
        pts=np.append(pts,[c,x,y,h])
        pts=pts.reshape([-1,4])
        c=c+1
        
    pts=np.append(pts,[c,goal[0],goal[1],goal[2]])    
    pts=pts.reshape([-1,4])
    
def collisionCheck(p1, p2):#check if path is collision free
    x1, y1 = p1
    x2, y2 = p2
    m = (y2 - y1)/(x2 - x1)
    c = y2 - (x2 * m)
    k = 0.01
    while k<1.0:
        x = x1+ k*(x2 - x1)
        y = y1+k*(y2 - y1)
        if (validNode(x, y)==False):
            return True
        k += 0.01
    return False


# In[3]:

#read inputs
node=pd.read_csv('nodes.csv', sep=',',header=None,comment='#')
node=node.to_numpy()
obs=pd.read_csv('obstacles.csv', sep=',',header=None,comment='#')
obs=obs.to_numpy()
init=tuple(node[0,1:4])
goal=tuple(node[-1,1:4])
k=100 #no of nodes
nbrs=1;#no of neighbours
pts=np.array([[]]);
genNode()
edge=np.array([[]]);


# In[4]:


def findNeighbours(ID):
    node1=(pts[ID-1,1],pts[ID-1,2])
    global nbrs
    global edge
    nb=nbrs
    for i in range(pts.shape[0]):
        if(node1==(pts[i,1],pts[i,2])):
            continue
        if(collisionCheck(node1,(pts[i,1],pts[i,2]))):
            continue
        #if(nb==0):
         #   break
        
        node2=(pts[i,1],pts[i,2])
        
        d=((node1[0]-node2[0])**2+(node1[1]-node2[1])**2)**0.5
        #if((ID,(i+1)) in edge[:,0:2]):
         #   continue
        
        ID2=pts[i,0]
        #pos=np.where(pts[:,0]==i)[0][0]
        
        edge=np.append(edge,(ID2,ID,d))
        edge=edge.reshape([-1,3])
        nb=nb-1


# In[5]:


for i in range(1,pts.shape[0]):
    findNeighbours(i)


# In[6]:

#savee path
np.savetxt("nodes.csv", pts, delimiter=",",fmt='%d,%f,%f,%f')
np.savetxt("edges.csv",edge,delimiter='=',fmt='%d,%d,%f')


# In[7]:

##
##Astar algo below
#functions
def checkNeighbour(ID):
    nbr=np.array([[]])
    for i in range(edge.shape[0]):
        if ID==edge[i,1]:
            if(checkInvalid(edge[i,0])):
                continue
            nbr=np.append(nbr,(edge[i,0],edge[i,2]))
            nbr=nbr.reshape(-1,2)
    return nbr

def checkInvalid(ID):
    if ID in CLOSED:
        return True;
    return False;
def updatePastCost(node1,parent1):
    past_cost[int(node1)-1]=findEdgeCost(parent1,node1)+past_cost[int(parent1)-1]
    
def findEdgeCost(node1,node2):
    for i in range(edge.shape[0]):
        if((node1 ==edge[i,1]) and (node2 ==edge[i,0])):
            return edge[i,2]
def getPath():
    i=-1;
    path=np.array([])
    path=np.append(path,goal)
    while(True):
        i=int(parent_node[i])
        path=np.append(path,i)
        i=i-1
        if(i==0):
            return path
    
edge=pd.read_csv('edges.csv', sep=',',header=None,comment='#')#read data
node=pd.read_csv('nodes.csv', sep=',',header=None,comment='#')
node=node.to_numpy()
edge=edge.to_numpy()
OPEN = np.array([[]])#initializing values
CLOSED = np.array([])
path=np.array([])
goal=node[-1,0]
init=node[0,0]
octg=node[:,3]
parent_node=np.empty_like(octg)*0-1
past_cost=np.empty_like(octg);
past_cost[0]=0
est_cost=past_cost.copy()
est_cost[0]=0

current=init;
while(True):
    nbr=checkNeighbour(current)#current
    
    for i in range(nbr.shape[0]):
        if(nbr.shape[1])==0:
            break;
            
        nbr[i,1]=nbr[i,1]+past_cost[int(current-1)]+octg[int(nbr[i,0]-1)]#update est_cost
    
    for i in range(nbr.shape[0]):#update node if updated cost is less than previos computation
        if(nbr.shape[1])==0:
            break;
        if nbr[i,0] in OPEN:
            
            OPEN=OPEN.reshape(-1,2)
            j=np.where(OPEN[:,0]==nbr[i,0])[0][0]
            if nbr[i,1]<OPEN[j,1]:
                OPEN=np.delete(OPEN,j,axis=0)
                OPEN=np.append(OPEN,nbr[i,:])
                parent_node[int(nbr[i,0]-1)]=current
                updatePastCost(nbr[i,0],current)
                est_cost[int(nbr[i,0]-1)]=nbr[i,1]
        else:
            OPEN=np.append(OPEN,nbr[i,:])
            parent_node[int(nbr[i,0]-1)]=current
            updatePastCost(nbr[i,0],current)
            est_cost[int(nbr[i,0]-1)]=nbr[i,1]
            OPEN=OPEN.reshape(-1,2)
    CLOSED=np.append(CLOSED,current)
    #print(OPEN)
    OPEN=OPEN.reshape(-1,2)
    OPEN=OPEN[OPEN[:,1].argsort()]
    current=OPEN[0,0];

    
    if(OPEN[0,0]==goal):
        break;   
   
    OPEN=np.delete(OPEN,[0,1])
    
path=getPath() #save soln
path=path.astype(int)
path=path.reshape((1,-1))
path=np.flip(path)
np.savetxt("path.csv", path, delimiter=",",fmt='%d')


# In[ ]:




