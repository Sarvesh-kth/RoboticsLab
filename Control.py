# import matplotlib.pyplot as plt
import numpy as np
from dubins import *

def neighbouradd(mini,minj,i,j,min,vset,cset,cost,xnum,ynum,nodes,parent,trcost,angle,angq):
   
   if i < xnum and i>= 0 and j < ynum and j >=0:
      if (i,j) not in cset and nodes[i][j] !=1:
         if(i,j) not in vset:
            vset.add((i,j))
            
         if cost[i,j,1] == 0 or cost[i,j,1] > min:
            cost[i,j,1] = cost[mini,minj,1] + trcost
            parent[i,j] = (mini,minj)
         angq[i,j] = angle



def solution(car):
  obj = car.obs
  ng = 3.5
  xnum = int(((car.xub - car.xlb)*ng) ) + 1
  ynum = int(((car.yub - car.ylb)*ng) )

  nodes = np.zeros((xnum,ynum))
  x0 = car.x0
  y0 = car.y0
  xt = car.xt
  yt = car.yt
  count = 0


  # Check for collision
  for i in range(0,xnum):
     for j in range(0,ynum):
        xc = i/ng
        yc = j/ng
        for ob in obj:
           if (xc - ob[0])**2 + (yc - ob[1])**2 <= ob[2]**2:
              nodes[i][j] = 1
              
              if j + 1 < ynum:
                  nodes[i][j+1] = 1

              if j - 1 >= 0:
                  nodes[i][j-1] = 1
                  
              if i + 1 < xnum:
                 nodes[i+1][j] = 1
                 if j + 1 < ynum:
                    nodes[i+1][j+1] = 1
                 if j - 1 >= 0:
                    nodes[i+1][j-1] = 1
                    
              if i - 1 >= 0:
                  nodes[i-1][j] = 1
                  if j + 1 < ynum:
                    nodes[i-1][j-1] = 1
                  if j - 1 >= 0:
                    nodes[i-1][j-1] = 1

  
  Xs = int(x0*ng)
  Ys = int(y0*ng)
  Xg = int(xt*ng)
  Yg = int(yt*ng)
  cost = np.zeros((xnum,ynum,2))            
  xn = [Xs/ng,Xg/ng]
  yn = [Ys/ng,Yg/ng]
  
  nodes[Xs,Ys] = -1
  nodes[Xg,Yg] = 2
  angq = np.zeros((xnum,ynum))
  #heuristic
  for i in range(0,xnum):
     for j in range(0,ynum):
        xc = i/ng 
        yc = j/ng 
        cost[i,j,0] = np.sqrt((xt-xc)**2 + (yt-yc)**2)
  vset = set()
  cset = set()
  vset.add((Xs,Ys))
  bordercost = np.zeros((xnum,ynum))
  
  for i in range(0,xnum):
    for j in range(0,ynum):
      if j < 4 or j > ng*10 - 4:
        bordercost[i,j] = 5
  parent = np.empty((xnum, ynum), dtype=object)
  for i in range(0,xnum):
    for j in range(0,ynum):
      if nodes[i][j] == 1:
        if j + 1 < ynum:
            bordercost[i][j+1] = 5
        if j - 1 >= 0:
            bordercost[i][j-1] = 5
                  
        if i + 1 < xnum:
            bordercost[i+1][j] = 5
            
        if i - 1 >= 0:
            bordercost[i-1][j] = 5
  start = 0
  angle = 0
  while True :
     if len(vset) == 0:
        break
     min = 1000
     curset = vset
     for node in curset:
        i = node[0]
        j = node[1]
        if i == Xg and j == Yg:
         break
        if cost[i,j,0] + cost[i,j,1] + bordercost[i,j] < min:
          min = cost[i,j,0] + cost[i,j,1] + bordercost[i,j]
          mini = i
          minj = j
     if i == Xg and j == Yg:
        break  
     
     angle = angq[mini,minj] 
     if angle == 180:
       ang1 = - 135
     else:
       ang1 = angle + 45
     if angle == -135 :
       ang2 = 180
     else:
       ang2 = angle -45
     i = mini
     j = minj
     if angle == 0:
       x1 = i + 1
       y1 = j
       x2 = i + 1
       y2 = j + 1
       x3 = i + 1
       y3 = j - 1
       c1 = 1
       c2 = 1.5
       c3 = 1.5
     elif angle == 45:
       x1 = i + 1
       y1 = j + 1
       x2 = i 
       y2 = j + 1
       x3 = i + 1
       y3 = j
       c1 = 1.5
       c2 = 1
       c3 = 1
     elif angle == 90:
       x1 = i 
       y1 = j + 1
       x2 = i - 1
       y2 = j + 1
       x3 = i + 1
       y3 = j + 1
       c1 = 1
       c2 = 1.5
       c3 = 1.5
     elif angle == 135:
       x1 = i - 1
       y1 = j + 1
       x2 = i 
       y2 = j + 1
       x3 = i - 1
       y3 = j
       c1 = 1.5
       c2 = 1
       c3 = 1
     elif angle == 180:
       x1 = i - 1
       y1 = j 
       x2 = i - 1
       y2 = j - 1
       x3 = i - 1
       y3 = j + 1
       c1 = 1
       c2 = 1.5
       c3 = 1.5
     elif angle == -135:
       x1 = i - 1
       y1 = j - 1
       x2 = i 
       y2 = j - 1
       x3 = i - 1
       y3 = j
       c1 = 1.5
       c2 = 1
       c3 = 1
     elif angle == - 90:
       x1 = i 
       y1 = j - 1
       x2 = i + 1
       y2 = j - 1
       x3 = i - 1
       y3 = j - 1
       c1 = 1
       c2 = 1.5
       c3 = 1.5
     elif angle == - 45:
       x1 = i + 1
       y1 = j - 1
       x2 = i + 1
       y2 = j 
       x3 = i 
       y3 = j - 1
       c1 = 1.5
       c2 = 1
       c3 = 1
     
     neighbouradd(mini,minj,x1,y1,min,vset,cset,cost,xnum,ynum,nodes,parent,1,angle,angq)
     if start > 4:
      neighbouradd(mini,minj,x2,y2,min,vset,cset,cost,xnum,ynum,nodes,parent,1,ang1,angq)
      neighbouradd(mini,minj,x3,y3,min,vset,cset,cost,xnum,ynum,nodes,parent,1,ang2,angq)
    #  neighbouradd(mini,minj,mini-1,minj+1,min,vset,cset,cost,xnum,ynum,nodes,parent,1.5)
    #  neighbouradd(mini,minj,mini+1,minj-1,min,vset,cset,cost,xnum,ynum,nodes,parent,1.5)
    #  neighbouradd(mini,minj,mini-1,minj,min,vset,cset,cost,xnum,ynum,nodes,parent,1)
    #  neighbouradd(mini,minj,mini,minj-1,min,vset,cset,cost,xnum,ynum,nodes,parent,1)
    #  neighbouradd(mini,minj,mini-1,minj-1,min,vset,cset,cost,xnum,ynum,nodes,parent,1.5)
     start+=1
     cset.add((mini,minj))
     vset.remove((mini,minj))
     
  xtraj = []
  ytraj = []
  ftraj =[]
  npoint = 0
  while parent[i,j] != (Xs,Ys):
     
     ftraj.insert(0,(parent[i,j][0],parent[i,j][1]))
     xtraj.insert(0,(parent[i,j][0])/ng )
     ytraj.insert(0,(parent[i,j][1])/ng )
     xi = parent[i,j][0]
     yi = parent[i,j][1]
     i = xi
     j = yi
     
  
  
  xtraj.insert(0,(x0))
  xtraj.insert(-1,(xt))
  ytraj.insert(0,(y0))
  ytraj.insert(-1,(yt))
  ftraj.insert(0,(Xs,Ys))
  ftraj.insert(-1,(Xg,Yg))
  
  
  
 

     
     



  '''
  Your code below
  '''

  # initial state
 

  '''
  Your code above
  '''
  tl = [0.0]
  theta = 0.0
  cl = []
  curnode = (Xs,Ys)
  i = 1
  exec = 0
  (xcur,ycur) = (x0,y0)
  print(xt,yt,xcur,ycur,1/ng)
  print((xt - xcur < 1/(ng)) and (yt - ycur < 1/ng))
  while i < len(xtraj) and (20-xcur > 0.3 and exec < 5000 and not ((xt - xcur < 1/(ng)) and (abs(yt - ycur) < 1/ng))) :
    npoint += 1
    exec+=1
    if abs(xt - xcur) < 0.3 and abs(yt-ycur) < 0.3:
      ntp = (xt,yt)
    else:
      ntp = (xtraj[i],ytraj[i])
    tanx = ntp[0] - xcur
    tany = ntp[1] - ycur

    angle = np.arctan2(tany,tanx)
    angdiff = angle - theta
    if angdiff > 0:
      if angdiff > np.pi/4:
        control = np.pi/4
      else:
        control = np.pi/4
    elif angdiff < 0:
      if angdiff < -np.pi/4:
        control =  -np.pi/4
      else:
        control = -np.pi/4
    else:
      control = 0
    
    xcur, ycur, theta = step(car, xcur, ycur, theta, control,0.01)
    xnode = int(xcur * ng)
    ynode = int(ycur * ng)
    # if exec % 3 == 0:
    #     print(xcur,ycur,xtraj[i],ytraj[i]," angle ",angle,theta)
    t = 2/ng
    if ((xtraj[i] - xcur)**2 + (ytraj[i]- ycur)**2 < t**2) or npoint == 35:
      npoint = 0
      i+=1
    

    
    cl.append(control)
    tl.append(tl[-1] + 0.01)
    
  exec = 0
#   while (xcur,ycur) != (xt,yt) :
#     exec+=1
#     tanx = xt - xcur
#     tany = yt - ycur

#     angle = math.atan2(tany,tanx)
#     angdiff = angle - theta
#     if angdiff > 0:
#       control = math.pi/4
#     elif angdiff < 0:
#       control = -1* math.pi/4
#     else:
#       control = 0
    
#     xcur, ycur, theta = step(car, xcur, ycur, theta, control)
#     cl.append(control)
#     tl.append(tl[-1] + 0.01)
  # fig, ax = plt.subplots()
  # ax.set_xlim(0, 20)
  # ax.set_ylim(0, 10)
  # x_points = []
  # y_points = []
  # for i in range(nodes.shape[0]):
  #     for j in range(nodes.shape[1]):
  #         if nodes[i][j] == 1:
  #             x_points.append(i / ng)
  #             y_points.append(j / ng)
  

  # ax.scatter(x_points, y_points, color='r', s=30)  
  # ax.scatter(xn, yn, color='g', s=40)
  # ax.scatter(xtraj, ytraj, color='b', s=10)
  # ax.set_aspect('equal', adjustable='box')

  # plt.grid(True)
  # print(xtraj[-1],ytraj[-1],xtraj[-2],ytraj[-2])
  # plt.show()
  return cl, tl