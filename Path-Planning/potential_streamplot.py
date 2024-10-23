
import numpy as np
import matplotlib.pyplot as plt




# creating two evenly spaced array with ranging from 
# -10 to 10




def add_goal(X,Y,s,r,loc):

  delx = np.zeros_like(X)
  dely = np.zeros_like(Y)


  for i in range(len(x)):
    for  j in range(len(y)):

      d = np.sqrt((loc[0] - X[i][j])**2 + (loc[1] - Y[i][j])**2)
      theta = np.arctan2((loc[1] - Y[i][j]),(loc[0] - X[i][j]))

      if (d < r):
        delx[i][j] = 0
        dely[i][j] = 0

      elif d > r+s:
          delx[i][j] = 50*s*np.cos(theta)
          dely[i][j] = 50*s*np.sin(theta)

      else:
          delx[i][j] = 50*(d-r)*np.cos(theta)
          dely[i][j] = 50*(d-r)*np.sin(theta)

  return delx,dely

x = np.arange(0,50,1)
y = np.arange(0,50,1)

s = 7
r=2
seek_points = np.array([[0,0]]) 
X, Y = np.meshgrid(x,y)
goal = [40,40]
delx, dely =add_goal(X, Y,s, r , goal)



delx , dely = add_goal(X,Y,s,r,goal)


def add_obstacle(X,Y,delx,dely,goal):
  obstacle = [25,25] 
  s=5
  r = 1
  for i in range(len(X)):
    for j in range(len(Y)):
      
      d_goal = np.sqrt((goal[0] - X[i][j])**2 + (goal[1] - Y[i][j])**2)
      d_obstacle = np.sqrt((obstacle[0] - X[i][j])**2 + (obstacle[1] - Y[i][j])**2)
      #print(f"{i} and {j}")
      theta_goal  = np.arctan2((Y[i][j] - goal[1]),(X[i][j] - goal[0]))
      theta_obstacle = -3.14 +np.arctan2((Y[i][j] - obstacle[1]),(X[i][j] - obstacle[0]))
      print(theta_obstacle)
      # using the Formula of avoiding obstacle
      if d_obstacle < r:
        delx[i][j] = -1*np.sign(np.cos(theta_obstacle))*5 +0
        dely[i][j] = -1*np.sign(np.cos(theta_obstacle))*5  +0
      elif d_obstacle>r+s:
        delx[i][j] += 0 -(50 * s *np.cos(theta_goal))
        dely[i][j] += 0 - (50 * s *np.sin(theta_goal))
      elif d_obstacle<r+s :
        delx[i][j] += -150 *(s+r-d_obstacle)* np.cos(theta_obstacle)
        dely[i][j] += -150 * (s+r-d_obstacle)*  np.sin(theta_obstacle) 
      if d_goal <r+s:
        if delx[i][j] != 0:
          delx[i][j]  += (50 * (d_goal-r) *np.cos(theta_goal))
          dely[i][j]  += (50 * (d_goal-r) *np.sin(theta_goal))
        else:
          
          delx[i][j]  = (50 * (d_goal-r) *np.cos(theta_goal))
          dely[i][j]  = (50 * (d_goal-r) *np.sin(theta_goal))
          
      if d_goal>r+s:
        if delx[i][j] != 0:
          delx[i][j] += 50* s *np.cos(theta_goal)
          dely[i][j] += 50* s *np.sin(theta_goal)
  
        else:
          
          delx[i][j] = 50* s *np.cos(theta_goal)
          dely[i][j] = 50* s *np.sin(theta_goal) 
      if d_goal<r:
          delx[i][j] = 0
          dely[i][j] = 0
      

  return delx,dely,obstacle,r



def plot_graph(X, Y, delx, dely,obj, fig, ax, loc,r,i, color,start_goal=np.array([[0,0]])  ):
  
  ax.quiver(X, Y, delx, dely)
  ax.add_patch(plt.Circle(loc, r, color=color))
  ax.set_title(f'Robot path with {i} obstacles ')
  ax.annotate(obj, xy=loc, fontsize=10, ha="center")
  return ax

for i in range(11):
  fig, ax = plt.subplots(figsize = (10,10))

  goal = [40,40]
  delx, dely =add_goal(X,Y,s,r,goal)
    
  plot_graph(X, Y, delx, dely , 'Goal',fig, ax, goal, 7,0, 'b' )
  obstacle_goal = [25,25] 
  delx, dely, loc, r = add_obstacle(X,Y, delx,dely,obstacle_goal)
  plot_graph(X, Y, delx, dely , 'Obstacle',fig, ax, obstacle_goal, r , 0,'m')
    #ax.add_patch(plt.Circle(loc, 2, color='m'))
  ax.streamplot(X,Y,delx,dely, start_points=seek_points,linewidth=4, cmap='autu')
    
  plt.show()