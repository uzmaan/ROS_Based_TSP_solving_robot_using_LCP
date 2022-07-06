import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64 
from std_msgs.msg import String
from std_msgs.msg import Int64MultiArray
import random
import numpy as np
import math
import time
global ackr
ackr =String()
ackr = '3'
l=0
class Node1(Node):
    def __init__(self):
        l=0
        super().__init__("Path_follower_node")
        self.path_publisher_ = self.create_publisher(Int64MultiArray, 'path_transmitter', 10)
        self.ack_publisher_ = self.create_publisher(Int64, 'ack_transmitter', 10)
        self.ack_reciever_ = self.create_subscription(String, 'ack_reciever',self.control, 10)
        self.publish_timer_ = self.create_timer(2.0, self.publish_array)
        print("init")
        
    def control(self,msg):
        global ackr
        global l
        x=Int64()
        ackr=msg.data
        x.data=5
        self.ack_publisher_.publish(x)
        print(ackr)
        print(x)
        print("init")
    
    def publish_array(self):
        global xtlp
        xtlp=xtlp.astype(np.int32)
        msg=Int64MultiArray()
        print(xtlp[1])
        list1 = xtlp.tolist()
        print(list1)
        ack=Int64()
        ack.data=0
        self.ack_publisher_.publish(ack)
        print("comms started")
        time.sleep(1)
        ack.data=int('1')
        self.ack_publisher_.publish(ack)   
        msg.data =list1[0]
        self.path_publisher_.publish(msg)
        ack.data=int('2')
        time.sleep(1)
        self.ack_publisher_.publish(ack)
        i=0
        time.sleep(1)
        ack.data=int('3')
        self.ack_publisher_.publish(ack)   
        msg.data =list1[1]
        self.path_publisher_.publish(msg)
        ack.data=int('4')
        time.sleep(1)
        self.ack_publisher_.publish(ack)
    
        
def main(args=None):
    rclpy.init(args=args)
    node = Node1()
    rclpy.spin(node)
    rclpy.shutdown()
# if __name__ == "__main__":
#     main()
time1=time.time()
time3=time.process_time()
def lcp(x):
    xdiff = int(findMinDiff(x[0]))
    xn = np.array(x)
    xs = xn[:, xn[0].argsort()]
    xcp=xs[1][0]
    lx=np.array(xcp)
    ly=np.array(xs[0][0])
    xm=np.amax(xs[1])
    xb=np.amin(xs[1])
    imr=0
    for i in range(0,len(xs[0])):       ##first pass from left to top
        if xs[1][i]>xcp and xs[1][i]!=xm :
            lx=np.append(lx,(xs[1][i]))
            ly=np.append(ly,(xs[0][i]))       
            xcp=xs[1][i]
        elif xs[1][i]==xm:
            lx=np.append(lx,(xs[1][i]))
            ly=np.append(ly,(xs[0][i]))       
            xcp=xs[1][i]
            imr=i
    xcp=xs[1][len(xs[0])-1]       
    lxr=np.array(xcp)
    lyr=np.array(xs[0][len(xs[0])-1])
    for i in range((len(xs[0])-1),imr-1,-xdiff):       ##second pass from right to top
        if xs[1][i]>xcp:
            lxr=np.append(lxr,(xs[1][i]))
            lyr=np.append(lyr,(xs[0][i]))       
            xcp=xs[1][i]
    lxr=lxr[::-1]
    lyr=lyr[::-1]
    for i in range(0,len(lxr)):
        lx=np.append(lx,(lxr[i]))
        ly=np.append(ly,(lyr[i]))       
    for i in range((len(xs[0])-1),0,-xdiff):       ##third pass from left to bottom
        if xs[1][i]<xcp and xs[1][i]!=xb:
            lx=np.append(lx,(xs[1][i]))
            ly=np.append(ly,(xs[0][i]))       
            xcp=xs[1][i]
        elif xs[1][i]==xb:
            lx=np.append(lx,(xs[1][i]))
            ly=np.append(ly,(xs[0][i]))       
            xcp=xs[1][i]
            imr=i
    xcp=xs[0][0]
    lxr=np.array(xcp)
    lyr=np.array(xs[0][0])
    for i in range(0,imr):       ##4th pass from right to bottom
        if xs[1][i]<xcp :
            lxr=np.append(lxr,(xs[1][i]))
            lyr=np.append(lyr,(xs[0][i]))       
            xcp=xs[1][i]
    lxr=lxr[::-1]
    lyr=lyr[::-1]
    for i in range(0,len(lxr)):
        lx=np.append(lx,(lxr[i]))
        ly=np.append(ly,(lyr[i]))       
    lx=np.append(lx,(xs[1][0]))
    ly=np.append(ly,(xs[0][0]))
    return lx,ly

def line2mat(xtf):
    xtfd = xtf.reshape(int(len(xtf)/2), 2)
    xtfp=np.transpose(xtfd)
    xtfd=xtfd.astype(np.int32)
    xtfp=xtfp.astype(np.int32)
    return xtfd,xtfp    

def findclosestline(line,point):
    #mi=False
    #m1i=False
    eqn1=0
    line=delsim(line)
    for j in range(0,len(point)):
        x3,y3=point[j][0],point[j][1]
        for i in range(0,len(line)-1):
            y2,y1,x2,x1=line[i+1][1],line[i][1],line[i+1][0],line[i][0]
            # print(line[i],line[i+1]) #(y1-y2/x1-x2 )
            if x2-x1!=0:           #CHECK FOR VERTICAL LINE
                m = (y2-y1)/(x2-x1)     ####slopification
                #mi=False
                if m!=0:
                    m1=-1/m
                    #m1i=False      #reverse slope
                #elif m==0:
                    #m1i=True     #reverse slope is infinite
                    
                if m==0:
                    eqn1=0  ############horizontal line 
                else:
                    eqn1=2 ########### angle line
            else:
                #mi=True           #normal slope vertical
                m1=0              #reverse slope horizontal
                eqn1=1 ################vertical line
            
            if eqn1==2:      #y1y2y3x1x2x3m1m2b1b2
                m1=-1/m
                b1=y3-(m1*x3)
        
def findclosestpoints(line,point,n):
    xp,yp=point[n][0],point[n][1]
    line=delsim(line)
    y2,y1,x2,x1=line[1][1],line[0][1],line[1][0],line[0][0]
    dis1=((yp-y1)**2)+((xp-x1)**2)
    dis1=math.sqrt(dis1)
    dis2=((yp-y2)**2)+((xp-x2)**2)
    dis2=math.sqrt(dis2)
    dismin=dis1+dis2
    for i in range(0,len(line)-1):
        y2,y1,x2,x1=line[i+1][1],line[i][1],line[i+1][0],line[i][0]
        dis1=((yp-y1)**2)+((xp-x1)**2)
        dis1=math.sqrt(dis1)
        dis2=((yp-y2)**2)+((xp-x2)**2)
        dis2=math.sqrt(dis2)
        dist=dis1+dis2
        if dist<dismin:
            j=i
            dismin=dist
    return j
def findMinDiff(arr):
    n = len(arr)
    diff = 10**20
    for i in range(n-1):
        for j in range(i+1, n):
            if abs(arr[i]-arr[j]) < diff and abs(arr[i]-arr[j]) >= 1:
                diff = abs(arr[i]-arr[j])
                # print(diff)

    return diff

def area(x1, y1, x2, y2, x3, y3):
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1)+ x3 * (y1 - y2)) / 2.0)

def isInside(x1, y1, x2, y2, x3, y3, x, y):
    A = area (x1, y1, x2, y2, x3, y3)
    A1 = area (x, y, x2, y2, x3, y3)
    A2 = area (x1, y1, x, y, x3, y3)
    A3 = area (x1, y1, x2, y2, x, y)
    if(A == A1 + A2 + A3):
        return True
    else:
        return False
def trianglise(xod):
    xo1len=len(xod)   #length of output
    # print(xo1len)
    xod=xod.astype(np.int32)
    xto=np.empty((0,2))
    x1t=np.empty(0)
    x2t=np.empty(0)
    x3t=np.empty(0)
    xa=[]
    for i in range(0,xo1len-1):
        x1t=[xod[i][0],xod[i][1]]   #first coordinate
        if (((xod[i+1][0]) > (xod[i][0])) and ((xod[i+1][1]) > (xod[i][1]))):
            x2t=[xod[i+1][0],xod[i][1]]    # first quadrant
        elif (((xod[i+1][0]) < (xod[i][0])) and ((xod[i+1][1]) < (xod[i][1]))):
            x2t=[xod[i+1][0],xod[i][1]]    # third quadrant
        elif (((xod[i+1][0]) > (xod[i][0])) and ((xod[i+1][1]) < (xod[i][1]))):
            x2t=[xod[i][0],xod[i+1][1]]    # second quadrant        
        elif (((xod[i+1][0]) < (xod[i][0])) and ((xod[i+1][1]) > (xod[i][1]))):
            x2t=[xod[i][0],xod[i+1][1]]    # fourth quadrant

        x3t=[xod[i+1][0],xod[i+1][1]]    #3rd coordinate
        xa=np.vstack((x1t,x2t,x3t))
        xto=np.concatenate((xto,xa),axis=0)
    return xto
######################

######################
def delsim(array):
    arr=np.empty(0)
    l=len(array)
    i=0
    # print(array)
    for i in range(0,l-1):
        if array[i][0]==array[i+1][0] and array[i][1]==array[i+1][1]:
            # print(array[i])
            nonethe=0
        else:
            arr=np.append(arr,(array[i][0],array[i][1]))
    arr=np.append(arr,(array[0][0],array[0][1]))
    xrmd,xrmp=line2mat(arr)
    return xrmd
def findinarray(x,y,array):
    for i in range(0,len(array)):
        if array[i][0]==x:       # find matching x
            if array[i][1]==y:   # find matching y
                return True
                break
    return False    
        
x = [[1, 2, 3, 4, 5, 6, 7, 8, 9, 2, 4, 4, 6, 7,7,5,3,4,3],
      [1, 3, 9,-4, 9,-6, 4,-2, 5, 1, 9, 3, 3, 2,7,0,0,3,4]]
#x = [[1, 2, 3,  6, 7, 8, 9, 2, 4, 6, 7,5],[1, 3, 9, -6, 4,-2, 5, 1, 3, 3, 2,0]]
lx,ly=lcp(x)
xo=np.array([ly,lx])
xod = np.transpose(xo)
x1 = xo[:, xo[0].argsort()]
xl = np.array([[9, 1], [1, 1]])
xo1len=len(xod)   #length of output
xod=xod.astype(np.int32)
#############################################Trianglisitation

#################################################
xto= trianglise(xod)
xtod=xto
xto = xto.reshape(int(len(xto)), 2)
xto=np.transpose(xto)
xto=xto.astype(np.int32)
xtod=xtod.astype(np.int32)

xn=np.array(x)
xnd=np.transpose(xn)         # code to remove similar elements to be added 
xrm=np.empty(0)
####################subtract array elements
len1=len(xnd)
for i in range(0,len1):
    lv=findinarray(xnd[i][0],xnd[i][1], xod)
    if lv== False:
        xrm=np.append(xrm,[int(xnd[i][0]),int(xnd[i][1])]) # adding missing elements
xrmd,xrmp=line2mat(xrm)
########################### point inside test
len2=int(len(xto[0])/3)
len1=len(xnd)
xtf=np.empty(0)
xvf=np.empty(0)
########################
for i in range(0,len2):
    j=i*3
    xtf=np.concatenate((xtf,xtod[j]),axis=0)
    for k in range(0,len(xrmd)):
        j=i*3
        px = xtod[j][0]
        py = xtod[j][1]
        qx = xtod[j+1][0]
        qy = xtod[j+1][1]
        rx = xtod[j+2][0]
        ry = xtod[j+2][1]
        sx,sy = xrmd[k][0],xrmd[k][1]
        if isInside(px,py,qx,qy,rx,ry,sx,sy)== True:
            xvf=np.array([sx,sy])
            xtvf = xtf.reshape(int(len(xtf)/2), 2)
            lv=findinarray(sx, sy, xtvf)
            if lv== False:
                xtf=np.concatenate((xtf,xvf),axis=0)
xtf=np.append(xtf,[int(xtf[0]),int(xtf[1])]) # adding last elements
xtfd = xtf.reshape(int(len(xtf)/2), 2)
xtfd=xtfd.astype(np.int32)
xtfp=np.transpose(xtfd)
#################################
xrm2=np.empty(0)
for i in range(0,len1):
    lv=findinarray(xnd[i][0],xnd[i][1], xtfd)
    if lv== False:
        xrm2=np.append(xrm2,[int(xnd[i][0]),int(xnd[i][1])]) # adding missing elements
xrm2d,xrm2p=line2mat(xrm2)
for i in range(0,len(xrm2d)):
    j=findclosestpoints(xtfd,xrm2d,i)
    xtfd=np.insert(xtfd,j+1,xrm2d[i],axis=0)
###########################################################
xtld=xtfd
xtlp=xtfd.transpose()
# plt.scatter(x[0], x[1], label="stars", color="green", s=10)
# plt.plot(xto[0], xto[1], label="stars", color="green")
# plt.show()

#lcp

time2=time.time()
time4=time.process_time()

print(time2-time1)
print(time4-time3)
print(xtlp)
main()