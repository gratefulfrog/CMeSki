#! /usr/bin/python
# algos.py
""""
Some algorithm work for vision and so on
"""

from math import *

def runTests():
    """
    The test cases:
    if the observer is at (0,0,0):
    * if the targetY <= ObserverY, FAIL
    * if the targetX or targetZ are outside the Observer field of view, FAIL
    * computed dy = O(y) - T(y), if not FAIL
    * computed dx = O(x) - T(x), if not FAIL
    * computed dz = 0(z) - T(z), if not FAIL
    Test Values:
    * Obs = (0,0,0)
    """
    b = .20  # pixies are separated by 20cm
    Targets= [(-16,-26,-5),  # targets are +/-16m off x-axis
              (+16,-26,-5), # 26m away on y-axis
              (-16,-26,5),   # +/-5m off z-axis
              (+16,-26,5)]

    testNumber = 0
    epsilon = 0.001  # for equality
    for Target in Targets:
        al=atan2(-Target[1],-Target[0]+b/2.0)
        ar=atan2(-Target[1],-Target[0]-b/2.0)
        ah=atan2(-Target[2],-Target[1])
        res= dxyzObserverTarget(b,al,ar,ah)
        print [round(r,2) for r in res]
        if all( abs(x-y)<epsilon for x,y in \
                    map(lambda a,b:(a,b),
                        Target,
                        res)):
            testNumber +=1
        else:
            print 'Test failed on: %s'%repr(Target)
            break
    print '%d successful tests'%testNumber


def doAssert(al,ar,ah=None):
    assert(all(((al>0.0),(ar>0.0),(pi>al),(pi>ar),
                any(((ah == None),(abs(ah) <= pi))))))
    
def at(al,ar):
    return ar-al

def L(b,al,ar):
    return b*sin(pi-ar)/sin(at(al,ar))

def R(b,al,ar):
    return b*sin(al)/sin(at(al,ar))

def d(b,al,ar):
    l = L(b,al,ar)
    return sqrt(pow(l,2) + \
                    pow((b/2.0),2) - \
                    l*b*cos(al))

def a(b,al,ar,dist=None):
    if not dist:
        dist = d(b,al,ar)
    print degrees(al),degrees(ar)
    print dist
    print R(b,al,ar)
    print sin(pi-ar)
    res = asin(R(b,al,ar)*sin(pi-ar)/dist)
    if ar>al>pi/2.0:
        res = pi -res
    return res

def dxyzObserverTarget(b,al,ar,ah):
    """ calculed such that the result is positve if the target is at 
    an x or y or z value less than the observer's values
    dz is calculated such that if the angle ah is positive then the distance
    in Z is positive,  ah is positive when observer is below target, ah is zero
    when observer is level with target, ah is negative when observer is above
    target.
    note that dy is always positive because of the assertions
    """
    doAssert(al,ar,ah)
    dist = d(b,al,ar)
    dy = -dist*sin(a(b,al,ar,dist))
    dx = abs(dist*cos(a(b,al,ar,dist)))
    if (al < pi-ar):
        dx=-dx
    dz = dy*tan(ah)
    
    return (dx, dy, dz)


PixieHorizontalField = radians(75.0)
PixieVerticalField = radians(47.0)
PixieHorizontalPixelRange = 640
PixieVerticalPixelRange = 400

class Pixie:
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

    def pixel2Anglep(self,(px,py)):
        return self.pixel2Angle(px,py)


    def pixel2Angle(self,px,py):
        """ return a pair (a,ah)
        """
        return (atan((px-self.centerX)/self.PDx),
                atan((py-self.centerY)/self.PDy))

    def test(self):
        #PixieHorizontalPixelRange
        #PixieVerticalPixelRange 
        #PixieHorizontalField
        #PixieVerticalField 
        epsilon = 0.001
        top = PixieVerticalPixelRange 
        topa = PixieVerticalField/2.0
        right = PixieHorizontalPixelRange 
        righta = PixieHorizontalField/2.0
        left =  bottom = 0
        lefta = -PixieHorizontalField/2.0
        bottoma = -PixieVerticalField/2.0
        centerH = PixieHorizontalPixelRange/2.0 
        centerV = PixieVerticalPixelRange/2.0
        centera = 0.0
        positionsResults = [
            ((left,bottom),(lefta,bottoma)),     #bottomLeft 
            ((centerH,bottom),(centera,bottoma)), #bottomCenter 
            ((right,bottom),(righta,bottoma)),    #bottomRight 
            ((right,centerV),(righta,centera)),   #centerRight 
            ((centerH,centerV),(centera,centera)), #center
            ((right,centerV),(righta,centera)),   #centerLeft 
            ((left,top),(lefta,topa)),        #topLeft
            ((centerH,top),(centera,topa)),      #topCenter
            ((right,top),(righta,topa))       #topRight
            ]
        def testForEquality(x,y):
            return (abs(x[0]-y[0]) < epsilon) and (abs(x[1]-y[1]) < epsilon) 
        return    all(testForEquality(calc,correct) for (calc,correct) in
                      map(lambda (pr,tr): (self.pixel2Angle(pr[0],pr[1]),tr),
                          positionsResults))

    # now we need a function that takes the oberserver(x,y,z)
    # and returns the pixel values for each pixie Left and Right (px,py)
    def observerTargetXYZ2PixiePxPy(self,target,observer):
        """ return the pixel pair (px,py)
        """
        dy = observer[1]- target[1]
        px = (((observer[0] + self.Xoffset-target[0]))*self.PDx/dy) + \
            self.centerX
        py  = (((observer[2]-target[2])*self.PDy)/dy) + self.centerY
        return (int(round(px,0)),int(round(py,0)))

# still need to test, DOES NOT seem good to go!!
# still an issue for the signs of dx, dy dz

class SimDrone:
    def __init__(self,b = 0.2):
        self.B=b
        self.pixieL = Pixie(-self.B/2.0)
        self.pixieR = Pixie(self.B/2.0)
        self.position = (0.0,0.0,0.0)


## still needs work...
    def dxyz(self,targetXYZ):
        (al,ahL) = self.pixieL.pixel2Anglep(
            self.pixieL.observerTargetXYZ2PixiePxPy(targetXYZ,
                                                    self.position))
        (ar,ahR) = self.pixieR.pixel2Anglep(
            self.pixieR.observerTargetXYZ2PixiePxPy(targetXYZ,
                                                    self.position))
        print "al:%f ar:%f ahL:%f ahR:%f"%(al,ar,ahL, ahR)
        return dxyzObserverTarget(self.B,((pi/2.0) -al),((pi/2.0)-ar),(ahL+ahR)/2.0)
    
