import numpy as np
import matplotlib.pyplot as plt
import turtle 
import time

#GLOBAL PARAMS
TIMER = 0
TIME_STEP = 0.05
SETPOINT = (0,0)
SIM_TIME = 500
INITIAL_X = -50
INITIAL_Y = -50
MASS = 1 #kg
MAX_THRUST = 50 #Newtons
F_y = 0 #y force
F_x = 0 #x force
V_y_i = 0 #initial y velocity
V_x_i = 0 #initial x velocity
X_i = 0 #initial x
Y_i = 0 #initial y
#------------
#---PID GAINS--- 
KU = 10
TU = 123 
# no overshoot
KP = KU * .2
KI = (.4 * KU)/TU
KD = (.066 * KU)/TU
#KD = 0.00128 for higher setpoints
antiWindup = True
# KP = 0.6
# KI = 0.0
# KD = 0.0
#---------------

class Simulation(object):
	def __init__(self):
		self.Insight = Rocket()
		self.pid = PID(KP,KI,KD,SETPOINT)
		self.screen = turtle.Screen()
		self.screen.setup(1280,900)
		self.marker = turtle.Turtle()
		self.marker.penup()
		self.marker.left(180)
		self.marker.goto(SETPOINT)
		self.marker.shape('circle')
		self.marker.resizemode("user")
		self.marker.shapesize(.5,.5,1)
		self.marker.color('red')
		self.sim = True
		self.timer = 0
		self.Xposes = np.array([])
		self.Yposes = np.array([])
		self.times = np.array([])
		self.Xkpe = np.array([])
		self.Xkde = np.array([])
		self.Xkie = np.array([])
		self.Xthrst = np.array([])
		self.Ykpe = np.array([])
		self.Ykde = np.array([])
		self.Ykie = np.array([])
		self.Ythrst = np.array([])
	def cycle(self):
		while(self.sim):
			Ythrust = self.pid.computeY(self.Insight.get_y())
			Xthrust = self.pid.computeX(self.Insight.get_x())
			self.Insight.set_ddy(Ythrust)
			self.Insight.set_dy()
			self.Insight.set_y()
			self.Insight.set_ddx(Xthrust)
			self.Insight.set_dx()
			self.Insight.set_x()
			#time.sleep(TIME_STEP)
			#print(f"Xthrust: {Xthrust:.2f} Ythrust: {Ythrust:.2f}")
			self.timer += 1
			if self.timer > SIM_TIME:
				print("SIM ENDED")
				self.sim = False
			elif self.Insight.get_y() > 700:
				print("OUT OF BOUNDS")
				self.sim = False
			elif self.Insight.get_y() < -700:
				print("OUT OF BOUNDS")
				self.sim = False
			elif self.Insight.get_x() > 700:
				print("OUT OF BOUNDS")
				self.sim = False
			elif self.Insight.get_x() < -700:
				print("OUT OF BOUNDS")
				self.sim = False
			self.times = np.append(self.times,self.timer)

			self.Xposes = np.append(self.Xposes,self.Insight.get_x())
			self.Xkpe = np.append(self.Xkpe,self.pid.get_Xkpe())
			self.Xkde = np.append(self.Xkde,self.pid.get_Xkde())
			self.Xkie = np.append(self.Xkie,self.pid.get_Xkie())
			self.Xthrst = np.append(self.Xthrst,Xthrust)

			self.Yposes = np.append(self.Yposes,self.Insight.get_y())
			self.Ykpe = np.append(self.Ykpe,self.pid.get_Ykpe())
			self.Ykde = np.append(self.Ykde,self.pid.get_Ykde())
			self.Ykie = np.append(self.Ykie,self.pid.get_Ykie())
			self.Ythrst = np.append(self.Ythrst,Ythrust)
		graph(self.times,self.Xposes,self.Xkpe,self.Xkde,self.Xkie,self.Xthrst, self.Yposes,self.Ykpe,self.Ykde,self.Ykie,self.Ythrst)

def graph(x,x1,x2,x3,x4,x5, y1,y2,y3,y4,y5):
	fig, (ax1, ax2,ax3,ax4,ax5,ax6,ax7,ax8,ax9,ax10) = plt.subplots(10, sharex=True)
	#fig.suptitle('antiwindup')
	ax1.set(ylabel='X pos')
	ax1.plot(x,x1)
	ax2.set(ylabel='XKP_e')
	ax2.plot(x,x2,'tab:red')
	ax3.set(ylabel='XKD_e')
	ax3.plot(x,x3,'tab:orange')
	ax4.set(ylabel='XKI_e')
	ax4.plot(x,x4,'tab:pink')
	ax5.set(ylabel='XThrust')
	ax5.plot(x,x5,'tab:brown')

	ax6.set(ylabel='Y pos')
	ax6.plot(x,y1, "tab:gray")
	ax7.set(ylabel='YKP_e')
	ax7.plot(x,y2,'tab:olive')
	ax8.set(ylabel='YKD_e')
	ax8.plot(x,y3,'tab:green')
	ax9.set(ylabel='YKI_e')
	ax9.plot(x,y4,'tab:purple')
	ax10.set(ylabel='YThrust')
	ax10.plot(x,y5,'tab:cyan')
	plt.show()

class Rocket(object):
	def __init__(self):
		global Rocket
		self.Rocket = turtle.Turtle()
		self.Rocket.shape('circle')
		self.Rocket.color('black')
		self.Rocket.resizemode("user")
		self.Rocket.shapesize(.5,.5,1)
		self.Rocket.penup()
		self.Rocket.goto(INITIAL_X,INITIAL_Y)
		self.Rocket.speed(0)
		#physics
		self.ddy = 0
		self.dy = V_y_i
		self.y = INITIAL_Y
		#physics
		self.ddx = 0
		self.dx = V_x_i
		self.x = INITIAL_X

	def set_ddx(self,thrust):
		self.ddx = F_x + thrust / MASS
	def get_ddx(self):
		return self.ddx
	def set_dx(self):
		self.dx += self.ddx * TIME_STEP
	def get_dx(self):
		return self.dx
	def set_x(self):
		self.Rocket.setx(self.x + TIME_STEP * self.dx)
	def get_x(self):
		self.x = self.Rocket.xcor()
		return self.x

	def set_ddy(self,thrust):
		self.ddy = F_y + thrust / MASS
	def get_ddy(self):
		return self.ddy
	def set_dy(self):
		self.dy += self.ddy * TIME_STEP
	def get_dy(self):
		return self.dy
	def set_y(self):
		self.Rocket.sety(self.y + TIME_STEP * self.dy)
	def get_y(self):
		self.y = self.Rocket.ycor()
		return self.y

class PID(object):
	def __init__(self,KP,KI,KD,target):
		self.Ykp = KP
		self.Yki = KI
		self.Ykd = KD
		self.Xkp = KP
		self.Xki = KI
		self.Xkd = KD 
		self.Xsetpoint = target[0]
		self.Ysetpoint = target[1]
		self.Yerror = 0
		self.Yintegral_error = 0
		self.Yerror_last = 0
		self.Yderivative_error = 0
		self.Youtput = 0
		self.Xerror = 0
		self.Xintegral_error = 0
		self.Xerror_last = 0
		self.Xderivative_error = 0
		self.Xoutput = 0

	def computeX(self, Xpos):
		self.Xerror = self.Xsetpoint - Xpos
		#self.Xintegral_error += self.Xerror * TIME_STEP
		self.Xderivative_error = (self.Xerror - self.Xerror_last) / TIME_STEP
		self.Xerror_last = self.Xerror
		self.Xoutput = self.Xkp*self.Xerror + self.Xki*self.Xintegral_error + self.Xkd*self.Xderivative_error
		if(abs(self.Xoutput)>= MAX_THRUST and (((self.Xerror>=0) and (self.Xintegral_error>=0))or((self.Xerror<0) and (self.Xintegral_error<0)))):
			if(antiWindup):
				#no integration
				self.Xintegral_error = self.Xintegral_error
			else:
				#if no antiWindup rectangular integration
				self.Xintegral_error += self.Xerror * TIME_STEP
		else:
			#rectangular integration
			self.Xintegral_error += self.Xerror * TIME_STEP
		if self.Xoutput >= MAX_THRUST:
			self.Xoutput = MAX_THRUST
		elif self.Xoutput <= -MAX_THRUST:
			self.Xoutput = -MAX_THRUST
		return self.Xoutput
	
	def computeY(self, Ypos):
		self.Yerror = self.Ysetpoint - Ypos
		#self.Yintegral_error += self.Yerror * TIME_STEP
		self.Yderivative_error = (self.Yerror - self.Yerror_last) / TIME_STEP
		self.Yerror_last = self.Yerror
		self.Youtput = self.Ykp*self.Yerror + self.Yki*self.Yintegral_error + self.Ykd*self.Yderivative_error
		if(abs(self.Youtput)>= MAX_THRUST and (((self.Yerror>=0) and (self.Yintegral_error>=0))or((self.Yerror<0) and (self.Yintegral_error<0)))):
			if(antiWindup):
				#no integration
				self.Yintegral_error = self.Yintegral_error
			else:
				#if no antiWindup rectangular integration
				self.Yintegral_error += self.Yerror * TIME_STEP
		else:
			#rectangular integration
			self.Yintegral_error += self.Yerror * TIME_STEP
		if self.Youtput >= MAX_THRUST:
			self.Youtput = MAX_THRUST
		elif self.Youtput <= -MAX_THRUST:
			self.Youtput = -MAX_THRUST
		return self.Youtput
		
	def get_Xkpe(self):
		return self.Xkp*self.Xerror
	def get_Xkde(self):
		return self.Xkd*self.Xderivative_error
	def get_Xkie(self):
		return self.Xki*self.Xintegral_error

	def get_Ykpe(self):
		return self.Ykp*self.Yerror
	def get_Ykde(self):
		return self.Ykd*self.Yderivative_error
	def get_Ykie(self):
		return self.Yki*self.Yintegral_error

def main():
	sim = Simulation()
	sim.cycle()

main()
