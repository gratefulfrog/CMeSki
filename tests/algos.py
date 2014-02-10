#! /usr/bin/python
# algos.py
""""
Some algorithm work for vision and so on
"""

from math import *

def testForEquality(x,y, epsilon = 0.001):
    return (abs(x-y) < epsilon) 


def runTests():
    """
    The test cases:
    if the observer is at (0,0,0):
    * if the targetY <= ObserverY, FAIL
    * if the targetX or targetZ are outside the Observer field of view, FAIL
    * computed dy = Obs(y) - Tar(y), if not FAIL
    * computed dx = Obs(x) - Tar(x), if not FAIL
    * computed dz = Obs(z) - Tar(z), if not FAIL
    Test Values:
    * Obs = (0,0,0)
    """
    B = .20  # pixies are separated by 20cm
    L = -6
    C =  0
    R =  6
    U =  4
    D = -4
    Y = 10
    Obs = (0,0,0)
    UL = (L,Y,U)
    UC = (C,Y,U)
    UR = (R,Y,U)
    CL = (L,Y,C)
    CC = (C,Y,C)
    CR = (R,Y,C)
    DL = (L,Y,D)
    DC = (C,Y,D)
    DR = (R,Y,D)
    def dxyz((x,y,z)):
        return (-x,-y,-z)
    def alarah(b,(x,y,z)):
        minus = atan2(abs(x)-abs(b/2.0),abs(y))
        plus  = atan2(abs(x)+abs(b/2.0),abs(y))
        ninety = pi/2.0
        if x<0:
            return (ninety+minus,ninety+plus,atan2(z,abs(y)))
        else:
            return (ninety-plus,ninety-minus,atan2(z,abs(y)))
    Targets = [UL,UC,UR,CL,CC,CR,DL,DC,DR]
    testNumber = 0
    epsilon = 0.001  # for equality
    print "target: ","Computed", "=", "Theoretical"
    for Target in Targets:
        (al,ar,ah) = alarah(B,Target)
        res= dXYZ(B,al,ar,ah)
        print [r for r in Target], ":",\
            [round(r,2) for r in res], "=", \
            [round(r,2) for r in dxyz(Target)]
        if all(testForEquality(x,y) for x,y in \
                   map(lambda a,b:(a,b),
                       dxyz(Target),
                       res)):
            testNumber +=1
        else:
            print 'Test failed on: %s'%repr(Target)
            break
    print '%d successful tests'%testNumber


def doAssert(al,ar,ah=None):
    assert(all(((al>0.0),(ar>0.0),(pi>al),(pi>ar),
                any(((ah == None),(abs(ah) <= pi/2.0))))))
    
def at(al,ar):
    return ar-al

def L(b,al,ar):
    return b*sin(ar)/sin(at(al,ar))

def R(b,al,ar):
    return b*sin(al)/sin(at(al,ar))

def d(b,al,ar):
    l = L(b,al,ar)
    return sqrt(pow(l,2) + 
                pow((b/2.0),2) - 
                l*b*cos(al))

def apPrime(b,al,ar,dist=None):
    if not dist:
        dist = d(b,al,ar)
    """print degrees(al),degrees(ar)
    print dist
    print R(b,al,ar)
    print sin(pi-ar)
    """
    a = asin(R(b,al,ar)*sin(ar)/dist)
    ap = (pi/2.0) - a
    if (al < pi-ar):
        apPrim = pi + abs(ap)
    else:
        apPrim = pi - abs(ap)
    return apPrim

def dXYZ(b,al,ar,ah):
    dist =  d(b,al,ar)
    apPrim = apPrime(b,al,ar,dist)
    dy = dist*cos(apPrim)
    return (dist*sin(apPrim),
            dy,
            dy*tan(ah))
            

PixieHorizontalField = radians(75.0)
PixieVerticalField = radians(47.0)
PixieHorizontalPixelRange = 640
PixieVerticalPixelRange = 400
class Pixie:
    """ we assume that (0,0) in the Pixie perspective is at lower left of 
    Pixie's view port
    """
    def __init__(self,offset):
        self.fx = PixieHorizontalField
        self.fy = PixieVerticalField
        self.rx = PixieHorizontalPixelRange
        self.ry = PixieVerticalPixelRange
        self.centerX = self.rx/2.0
        self.centerY = self.ry/2.0
        self.PDx = (self.centerX)/tan(self.fx/2.0)
        self.PDy = (self.centerY)/tan(self.fy/2.0)
        self.Xoffset = offset

    def nPx(self,px):
        return px - self.centerX
    def nPy(self,py):
        return py - self.centerY

    def pixel2Angles(self,px,py):
        """ return a pair (a,ah)
        """
        assert(px>=0 and px <=self.rx)
        assert(py>=0 and py <=self.ry)
        return ((pi/2.0) - atan2(self.nPx(px),self.PDx),
                atan2(self.nPy(py),self.PDy))
    
    def test(self):
        #PixieHorizontalPixelRange
        #PixieVerticalPixelRange 
        #PixieHorizontalField
        #PixieVerticalField 
        top = PixieVerticalPixelRange 
        topa = PixieVerticalField/2.0
        right = PixieHorizontalPixelRange 
        righta = pi/2.0 - PixieHorizontalField/2.0
        left =  bottom = 0
        lefta = pi/2.0 + PixieHorizontalField/2.0
        bottoma = -PixieVerticalField/2.0
        centerH = PixieHorizontalPixelRange/2.0 
        centerV = PixieVerticalPixelRange/2.0
        centeraV = 0.0
        centeraH = pi/2.0
        positionsResults = [
            ((left,bottom),(lefta,bottoma)),     #bottomLeft 
            ((centerH,bottom),(centeraH,bottoma)), #bottomCenter 
            ((right,bottom),(righta,bottoma)),    #bottomRight 
            ((right,centerV),(righta,centeraV)),   #centerRight 
            ((centerH,centerV),(centeraH,centeraV)), #center
            ((right,centerV),(righta,centeraV)),   #centerLeft 
            ((left,top),(lefta,topa)),        #topLeft
            ((centerH,top),(centeraH,topa)),      #topCenter
            ((right,top),(righta,topa))       #topRight
            ]

        return    all((testForEquality(calc[0],correct[0]) and \
                           testForEquality(calc[1],correct[1])) \
                          for (calc,correct) in \
                          map(lambda (pr,tr): 
                              (self.pixel2Angles(pr[0],pr[1]),tr),
                              positionsResults))

    # now we need a function that takes the oberserver(x,y,z)
    # and returns the pixel values for each pixie Left and Right (px,py)
    def simTargPxPy(self,targ,obs):
        """ return the pixel pair (px,py) for the Pixie
        for the target from the observer coords.
        """
        dxp = targ[0] - (obs[0] +self.Xoffset)
        dyp= targ[1] - obs[1]
        dzp = targ[2] - obs[2]
        px = self.PDx*dxp/dyp + self.centerX
        py = self.PDy*dzp/dyp +  self.centerY
        px = int(round(px,0))
        py = int(round(py,0))
        assert(px>=0 and px<=self.rx)
        assert(py>=0 and py<=self.ry)
        return (px,py)

    def simTargAngles(self,targ,obs):
        (px,py) = self.simTargPxPy(targ,obs)
        return self.pixel2Angles(px,py)

class SimDrone:
    def __init__(self,b = 0.2):
        self.B=b
        self.pixieL = Pixie(-self.B/2.0)
        self.pixieR = Pixie(self.B/2.0)
        self.position = (0.0,0.0,0.0)

    def dXYZ(self,targ):
        (al,ahL) = self.pixieL.simTargAngles(targ,self.position)
        (ar,ahR) = self.pixieR.simTargAngles(targ,self.position)

        #print "al:%f ar:%f ahL:%f ahR:%f"%(al,ar,ahL, ahR)
        return dXYZ(self.B,al,ar,(ahL+ahR)/2.0)
    
# this needs to be tested... seems very poor in Y-axis precision..
