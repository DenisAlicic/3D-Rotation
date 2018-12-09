import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from pyquaternion import Quaternion
from main import *
import warnings

TIMER_ID = 0
TIMER_INTERVAL = 25
t = 0
animation_ongoing = False
overall_time = 2.5


startCenter = np.array([0,0,0])
endCenter = np.array([2,2,-2])

startEuler = np.array([-np.arctan(np.pi / 2),-np.arcsin(np.pi / 4),np.arctan(np.pi / 3)])
endEuler = np.array([-np.arctan(np.pi),-np.arcsin(np.pi / 6),np.arctan(np.pi / 9)])

def Slerp(q1, q2, tm, t):
    warnings.filterwarnings("ignore")
    cos0 = float(np.inner(q1, q2))
    if cos0 < 0:
        q1 = -q1
        cos0 = -cos0
    if cos0 > 0.95:
        return q1
    phi0 = np.arccos(cos0)
    qs = (np.sin(phi0*(1-t/tm)) / np.sin(phi0)) * q1 + (np.sin(phi0*(t/tm)) / np.sin(phi0)) * q2
    return qs
def linearInterpolation(c1, c2, tm, t):
    c = (1-t/tm)*c1 + (t/tm)*c2
    return c

def Euler2Q(phi,theta,omega):
    return AngleAxis2Q(*AxisAngle(Euler2A(phi,theta,omega)))
def Q2Euler(q):
    return A2Euler(Rodrigez(*Q2AxisAngle(q)))
def animation():

    glPushMatrix()

    #glRotate3v()
    glColor3f(1,0,0)
    glutWireTeapot(1)
    glPopMatrix()
    return
def on_timer(id):
    global t,overall_time,animation_ongoing
    if id != TIMER_ID:
        return

    if t >= overall_time:
        animation_ongoing = False;
    else:
        t += .02

    glutPostRedisplay()

    if animation_ongoing:
            glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID)

def on_keyboard(key,x,y):
    global animation_ongoing
    if key == b'\x1b' or key == b'q' or key == b'Q':
        exit()
    if key == b' ':
        if animation_ongoing == False:
                glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID)
                animation_ongoing = True
    if key == b's' or key == b'S':
        animation_ongoing = False

def main():
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(800,800)
    glutCreateWindow("Slerp animation")

    glClearColor(0.2,0.2,0.2,1.)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    lightZeroPosition = [10.,4.,10.,1.]
    lightZeroColor = [0.8,1.0,0.8,1.0] #green tinged
    glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)
    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.1)
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.05)
    glEnable(GL_LIGHT0)
    glutDisplayFunc(display)
    glutKeyboardFunc(on_keyboard)
    glMatrixMode(GL_PROJECTION)
    gluPerspective(40.,1.,1.,40.)
    glMatrixMode(GL_MODELVIEW)
    gluLookAt(10,10,10,
              0,0,0,
              0,1,0)
    glPushMatrix()
    glutMainLoop()
    return
def animation():
    global t,overall_time
    q1 = Euler2Q(startEuler[0],startEuler[1],startEuler[2])
    q2 = Euler2Q(endEuler[0],endEuler[1],endEuler[2])
    q3 = Slerp(q1, q2, overall_time, t)
    phi, omega, theta = Q2Euler(q3)
    translation = linearInterpolation(startCenter,endCenter,overall_time,t)
    glPushMatrix()
    color = [1.0,0.,0.,1.]
    glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
    glTranslatef(translation[0],translation[1],translation[2])
    glRotatef(np.rad2deg(phi),1,0,0)
    glRotatef(np.rad2deg(omega),0,1,0)
    glRotatef(np.rad2deg(theta),0,0,1)
    glTranslatef(startCenter[0],startCenter[1],startCenter[2])
    glRotatef(np.rad2deg(startEuler[0]),1,0,0)
    glRotatef(np.rad2deg(startEuler[1]),0,1,0)
    glRotatef(np.rad2deg(startEuler[2]),0,0,1)
    glutWireTeapot(1)
    glPopMatrix()
    return
def display():
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    animation()
    glutSwapBuffers()
    return

if __name__ == "__main__":
    main()
