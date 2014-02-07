#! /usr/bin/python
# algos.py
""""
Some algorithm work for vision and so on
"""

from math import *

def runTests():
    b = 20  # pixies are separated by 20cm
    Targets= [(1600,2600,-500),  # targets are +/-16m off x-axis
              (-1600,2600,-500), # 26m away on y-axis
              (1600,2600,500),   # +/-5m off z-axis
              (-1600,2600,500)]

    testNumber = 0
    epsilon = 0.001  # for equality
    for Target in Targets:
        al=atan2(Target[1],Target[0]+b/2.0)
        ar=atan2(Target[1],Target[0]-b/2.0)
        ah=atan2(Target[2],Target[1])
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
    res = asin(R(b,al,ar)*sin(pi-ar)/dist)
    if ar>al>pi/2.0:
        res = pi -res
    return res

def dxyzObserverTarget(b,al,ar,ah):
    """ calculed such that the result is positve if the target is at 
    an x or y or z value greater than 0
    dz is calculated such that if the angle ah is positive then the distance
    in Z is positive,  ah is positive when observer is below target, ah is zero
    when observer is level with target, ah is negative when observer is above
    target.
    note that dy is always positive because of the assertions
    """
    doAssert(al,ar,ah)
    dist = d(b,al,ar)
    dy = dist*sin(a(b,al,ar,dist))
    return (dist*cos(a(b,al,ar,dist)), dy, dy*tan(ah))


def px2Angle(rg,f,px,yDir=False):
    """ based on pixel range (rg) and field of view angle f, return
    the angle corresponding to the px value px.
    the angle is positive to the right and up, i.e. assumes that (0,0)
    is in lower left corner of field of view.
    Angle is calculated for use as al,ar,ah above!
    """
    D =rg/(2.0*tan(f/2.0))
    if yDir:
         return atan2((px-rg/2.0), D)
    else:
         return pi/2.0- atan2((px-rg/2.0), D)


    
