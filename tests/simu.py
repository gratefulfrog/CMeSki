#! /usr/bin/python
# simu.py
# 2014 02 07
# simulation stuff for stero vision and navigation
"""

Flying algos:
Definitions:
Drone orientation: 
      drone heading is always 0deg relative to itself,
      cameras are looking directly backwards, at 180 to heading.
Drone Vy: 
      + if flying forward, relative to itself, 
      - if flying backwards.
Drone Vx: 
      + if flying to the right relative to itself, 
      - if flying left.
Drone Va: (angular velocity) 
      + if rotating counter clockwise as seen from above, 
      - if rotating clockwise as seen from above.
Drone Vz:
      + if climbing
      - if descending.

1. Init: the drone is launched with a view of the target. The target needs to 
         inform the drone has to the heading it should assume. One way would be
         to lauch at the right heading; another would be to use color coded
         targets on gloves so that the target can align to heading and the drone
         can orient so as to see the 2 targets symetrically, similar to seeking
         the wind direction.

In stereo vision we have the following parameters
Constants:
B: the base spearation between the 2 view ports (i.e. cameras); this is a fixed 
   constant value.
D: this is the predetermined optimal distance from tracker to target. The 
   distance is measured from target center to tracker center.
A1, A2: these are the predetermined angles right and left from below, i.e.
        when facing tracker with target behind, which correspond to the 
        predetermined distance D.
H: this is the predetermined height above top of traget. Note that the top
   of the target will be the head of the skier, so we can say that it is 
   already 2m above ground level.
AV: this is the predetermine vertical angle at which the target should appear
    so as to conform with H
Measured values:
a1, a2, av : the angles right and left and vertical, these are measured 
             at each cycle. 
d: this is the calculated actual distance.
h: this is the calculated actual height above target.


Flight Command values:
If a2 != 180-a1 epsilonX:
   inc(Vx, delta(a2, 180-a1), maxVx)
Else:
   zero(Vx)

If d != D, espilonY :
   inc(Vy, delta(d,D), maxVy)
Else:
   zero(Vy)

If h != H, espilonY :
   inc(Vz, delta(h,H), maxVz)
Else:
   zero(Vz)

"""

from math import *
import pid

class DroneVision:
    def __init__(self):
        self.B = 0.2  # meters
        self.D = 10.0   # meters
        self.H = 3.0 # meters
        self.A2 = atan(self.D/(self.B/2.0)) # radians
        self.A1 = pi - self.A2  # radians
        self.AV = atan(self.H/self.D) # radians
        #self.maxVx = 10.0 # meters/sec 36 km/hr
        #self.maxVy = 10.0 # meters/sec 36 km/hr
        #self.epsilonX = 0.5  # meters
        #self.epsilonY = 1.0   # meters
        self.d = 0.0
        self.dx = self.dy = self.dz =0.0

    def updateDH(self,a1,a2,av):
        """ these formulae are wrong due to looking from the perspectiv of
        the skier!!!
        """"
        sinPiMinusA1 = sin(pi -a1)
        sinA1MinusA2 = sin(a1 - a2)        
        L = self.B*sinPiMinusA1/sinA1MinusA2
        #print L
        Lsquared     = pow(L,2)
        Bsquared     = pow(self.B, 2)
        self.d = sqrt(Lsquared + 
                      Bsquared*(1.0/4.0 - sinPiMinusA1*cos(a2)/sinA1MinusA2))
        l = asin(min(1.0,max(-1.0,L*sin(a2)/self.d)))
        print "a1:",degrees(a2)        
        print "a2:",degrees(a2)
        print "l:",degrees(l)
        b = radians(90) - l
        print "b:",degrees(b)
        self.dy = self.d*cos(b)
        """oa1 = pi-a1
        if abs(a2)>abs(oa1):
            self.dx = -abs(self.d*sin(b))
        else:
            self.dx = abs(self.d*sin(b))
        """
        self.dx = -self.d*sin(b)
        self.dz = self.dy*tan(av)

"""
>>> import algos
>>> from math import *
>>> dv =algos.DroneVision()
>>> a = radians(89.99999999999999)
>>> dv.d(a,pi-a)
450359962737049.6 #meters...
>>> dv.h(10,radians(17),radians(17))
3.057306814586604  # meters

"""

class Drone:
    def __init__(self):
        self.dv = DroneVision()
        self.f = Flyer()
        self.vel = self.pos = [0.0,0.0,0.0]
        self.deltaP = [0,10.0,5.0]
        self.eyePos =[[-self.dv.B/2.0,0,0],
                      [+self.dv.B/2.0,0,0]]
        self.angles = [0,0,0]
        self.pids=[1,2,3]
        pvs = [0.6,0.4,0.3]
        for i in range(3):
            self.pids[i] = pid.PID(pvs[0],pvs[1],pvs[2])
            self.pids[i].setPoint(self.deltaP[i])

    def newEyePos(self):
        self.eyePos =[[self.pos[0]-self.dv.B/2.0, self.pos[1], self.pos[2]],
                      [self.pos[0]+self.dv.B/2.0, self.pos[1], self.pos[2]]]
    
    def newAngles(self, sk):
        """ update angles between self ans skier (argument)
        """
        self.newEyePos()
        self.angles = [atan2(sk.pos[1] - self.eyePos[0][1],
                             sk.pos[0] - self.eyePos[0][0]),
                       atan2(sk.pos[1] - self.eyePos[1][1],
                             sk.pos[0] - self.eyePos[1][0]),
                       atan2(sk.pos[2] - self.pos[2],
                             sk.pos[1] - self.pos[1])]

    def newDxDyDz(self,sk):
        self.newAngles(sk)
        self.dv.updateDH(self.angles[0],self.angles[1],self.angles[2])

    def newPidV(self):
        for i in range(3):
            self.vel[i] = self.pids[i].update(self.deltaP[i])
        
    def newV(self):
        ddRange =[0.01,0.2,2,5,10]
        offset =[0,10,5]
        vRange = [0,1,4,10,20]
        vMax = 50
        found = False
        for i in range(len(self.deltaP)):
            for j in range(len(ddRange)):
                if abs(offset[i] - self.deltaP[i]) < ddRange[j]:
                    self.vel[i] = vRange[j]*sign(offset[i] - self.deltaP[i])
                    #print 'assigned vel[%d] = %f' %(i,self.vel[i])
                    found = True
                    break
                if not found:
                    self.vel[i] = vMax*sign(offset[i]-self.deltaP[i])
                    
    def newVP(self,deltaT):
        self.pos = self.f.fly(self.pos,self.vel,deltaT)
        self.deltaP = [self.dv.dx, self.dv.dy,self.dv.dz]
        #self.newPidV()
        #self.newV()
        
        
    def update(self,sk, deltaT):
        self.newDxDyDz(sk)
        self.newVP(deltaT)
    
class Hill:
    def pitch(self,pos):
        # pos is a triplet x,y,z
        # return the pitch as a function of 3D position
        return radians(-50.0)

class Skier:
    def __init__(self, turns=False):
        self.V =  10.0 # m/s * 3.6 to get Km/h
        self.pos = [0.0,0.0,0.0]
        if turns:
            self.heading = radians(45.0)
        else:
            self.heading = radians(90.)
        self.turning = turns
        self.curTime = 0
        self.turnDeltaT = 2 # seconds
        self.lastTurnTime =0.0
        
    def ski(self,deltaT, hill):
        pitch = hill.pitch(self.pos)
        deltaD = self.V * deltaT
        self.pos[0] += deltaD * cos(self.heading) 
        y = deltaD * sin(self.heading) 
        self.pos[1] += y
        self.pos[2] +=  y * sin(pitch)
        self.curTime += deltaT
        if self.turning:
            self.turn()

    def turn(self):
        tDT = self.turnDeltaT
        if self.lastTurnTime == 0.0:
            tDT = self.turnDeltaT/2.0
        #print "lastTurnTime,curtime,tDT", self.lastTurnTime, self.curTime,tDT
        if self.curTime - self.lastTurnTime >= tDT:
            #print "turning"
            self.lastTurnTime = self.curTime
            if self.heading == radians(45.0):
                self.heading = radians(135.0)
            else:
                self.heading = radians(45.0)

        
class Flyer:
    def __init__(self):
        self.v = [0.0,0.0,0.0]

    def fly(self,pVec,vVec,deltaT):
        """returns a new position vector by applying velocity vector
        over delta-t"
        """
        return map(lambda p,v: p+v*deltaT, pVec,vVec)
            

def sign(x):
    if x != 0:
        return abs(x)/x
    else:
        return 1.0

def equalsEpsilon(a,b,e=0):
    return abs(a-b) < e

def rangeMap(v, r1, r2, t1, t2):
    if v <= r1:
        return t1
    elif v >= r2:
        return t2
    else:
        return (v - r1)*(t2 - t1)/(r2 - r1)


def skit(dtt, turns=False,hd=90):
    s = Skier(turns)
    s.heading = radians(hd)
    s.pos = [0,0,0]
    h = Hill()
    d = Drone()
    d.pos = [0,100,5]
    t = 0
    dt =0.1
    with open('data', 'w') as f:
        f.write('# time\tdx\tdy\tdz')
    while round(t,5) < round(dtt,5):
        t += dt 
        s.ski(dt, h)
        #print "Time", "skierX", "skierY", "skierZ", "skierHeading"
        #print t,[round(x,1) for x in (s.pos)], round(degrees(s.heading),1)
        #print 'time: %f' %(t)
        d.update(s,dt)
        #printD(d,s)
        with open('data', 'a') as f:
            #f.write('\n%f\t%f\t%f\t%f'%(t,s.pos[0],s.pos[1],s.pos[2]))
            f.write('\n%f\t%f\t%f\t%f'%(t,d.dv.dx,d.dv.dy,d.dv.dz))

def printD(d,sk):
    #self.newAngles(sk)
    #self.dv.updateDH(self.angles[0],self.angles[1],self.angles[2])
    print "droneX", "droneY","droneZ"
    print [round(x,2) for x in d.pos]
    """print "dronevX", "droneVY","droneVZ"
    print [round(x,2) for x in self.vel]
    """
    #print "droneDx to Skier", "droneDy to Skier","droneDz to Skier"
    print round(d.dv.dx,1), round(d.dv.dy,1), round(d.dv.dz,1)
    
