from math import sin, cos, sqrt, pi
import pylab
from FOC_main import FOC

# motor characteristics  i.e print(motor.maxTorque) prints 5 mNm



class maxon:
    def __init__(self):
        self.torque_constant = 51e-3
        self.R = 5.03
        self.L = 2.24e-3
        self.J = 92.5 * 1e-7 #rotor inertia
        self.friction = 3.723e-3 / 458.67
        
        self.maxTorque = 54.7e-3
        self.pole_pairs = 8

          # motor for simulation



class motor_gains:
    def __init__(self):

        self.d_gains = (0.4, 800.0)
        self.q_gains = (0.4, 800.0)
        
class motor_sim:
    def __init__(self,motor,start_position=0.0):
        self.motor = motor
        

        self.ia = [0.0]
        self.ib = [0.0]
        self.ic = [0.0]

        self.vxa = [0.0] #terminal voltage
        self.vxb = [0.0]
        self.vxc =[0.0]

        self.va = [0.0]  # phase  voltages
        self.vb = [0.0]
        self.vc = [0.0]

        self.emfA = [0.0]
        self.emfB = [0.0]
        self.emfC = [0.0]

        self.position = [start_position]

        self.velocity = [0.0]

        

        self.torque = [0.0]
        self.t = [0.0]

        

    def update(self, t, dt, vxa, vxb, vxc):
        motor = self.motor
        ia = self.ia[-1]
        ib = self.ib[-1]
        ic = self.ic[-1]


        # phase voltages
        vc = ( vxa + vxb + vxc ) / 3.0
        va = vxa - vc
        vb = vxb - vc
        vc = vxc - vc

        # kill non-zero circulating currents
       
        R = motor.R * 0.5
    
        Lva = va - R*ia - self.emfA[-1]
        Lvb = vb - R*ib - self.emfB[-1]
        Lvc = vc - R*ic - self.emfC[-1]

        L = motor.L * 0.5

        ia += Lva / L *dt
        ib += Lvc / L *dt
        ic += Lvc / L *dt

        iavg = (ia+ib+ic)/3.0
        
        ia -= iavg
        ib -= iavg
        ic -= iavg


        theta_el  = self.position[-1] * motor.pole_pairs

        sa = sin(theta_el)
        sb = sin(theta_el - pi*2/3)
        sc = sin(theta_el + pi*2/3)

        # note that the torque_constant is based on block commutation.
	# read notes about motor torque with sinusoidal control to
	# get a better idea scale constant comes from
        scale  = 0.60459978807807258  # pi/3 / sqrt(3)
        torque_constant = motor.torque_constant * scale

        

        


        
        # sum of current should be 0, kill non-zero (circulating) currents
       



        
        torque = (ia*sa + ib*sb + ic*sc) * torque_constant

         
        velocity = self.velocity[-1]

        velocity +=  (torque - velocity*motor.friction) / (motor.J) * dt
                
        position = self.position[-1] + velocity*dt




        emfA = motor.torque_constant * velocity *sa
        emfB =  torque_constant * velocity *sb
        emfC =  torque_constant * velocity * sc 
        

        
       


        # update values
        self.emfA.append(emfA)
        self.emfB.append(emfB)
        self.emfC.append(emfC)
        

        self.ia.append(ia)
        self.ib.append(ib)
        self.ic.append(ic)

        self.position.append(position)

        self.velocity.append(velocity)

        self.vxa.append(vxa) #terminal voltage
        self.vxb.append(vxb)
        self.vxc.append(vxc)

        self.torque.append(torque)
        self.t.append(t)

        self.va.append(va)  # phase  voltages
        self.vb.append(vb)
        self.vc.append(vc)
        return (position, velocity, ia, ib, ic) 
        







def simulation():
    motor = maxon()
    gains = motor_gains()
    d_gains = gains.d_gains
    q_gains = gains.q_gains


    motorSim = motor_sim(motor)

    supply = 24.0

    target_torque = 0.85*motor.maxTorque

    t = 0.0
    pwm_dt = 1.0 / 35e3 

    foc = FOC(pwm_dt,motor,d_gains,q_gains,1.2)
##    foc.reset(motorSim.velocity[-1], supply)
    el_angle_offset = 0.0 # -20.0 * pi/180.
    angle_offset = el_angle_offset / motor.pole_pairs



    pwm_t = []
    pwm_cmd_a = []
    pwm_cmd_b = []
    pwm_cmd_c = []

##
##    if True:
##        cmd_a,cmd_b,cmd_c = foc.update(t, 0.0, motorSim.position[-1],motorSim.velocity, 0., 0., 0.,supply)
##        vxa = supply*cmd_a
##        vxb = supply*cmd_b
##        vxc = supply*cmd_c
##    else:
##        vxa,vxb,vxc = (0.0, 0.0, 0.0)
##        cmd_a,cmd_b,cmd_c = (0.0, 0.0, 0.0)
##
   
    
    cmd_a,cmd_b,cmd_c = foc.update(t, 0.0, motorSim.position[-1],motorSim.velocity[-1], 0.0, 0.0, 0.0, supply)

    vxa = supply*cmd_a
    vxb = supply*cmd_b
    vxc = supply*cmd_c


    motor_sim_substeps = 8
    motor_sim_dt = pwm_dt/motor_sim_substeps
    runtime = 15e-3



    while t < runtime:
        for j in range(1):
            for i in range(motor_sim_substeps):
                position,velocity,ia,ib,ic = motorSim.update(t, motor_sim_dt , vxa,  vxb,  vxc)
                t+=motor_sim_dt
            # calulate command voltages here to simulate 1 cycle control loop delay
            vxa = supply*cmd_a
            vxb = supply*cmd_b
            vxc = supply*cmd_c
            pwm_t.append(t)
            pwm_cmd_a.append(cmd_a)
            pwm_cmd_b.append(cmd_b)
            pwm_cmd_c.append(cmd_c)


            
        cmd_a,cmd_b,cmd_c = foc.update(t, target_torque, position+angle_offset,velocity, ia,ib,ic,supply)

    
    ts = pylab.array(motorSim.t) * 1e3
    tc = pylab.array(foc.t) * 1e3

    va = pylab.array(motorSim.va)
    vb = pylab.array(motorSim.vb)    
    vc = pylab.array(motorSim.vc)
    backemf_a = pylab.array(motorSim.emfA)
    backemf_b = pylab.array(motorSim.emfB)
    backemf_c = pylab.array(motorSim.emfC)
    ia = pylab.array(motorSim.ia)
    ib = pylab.array(motorSim.ib)
    ic = pylab.array(motorSim.ic)
    vxa = pylab.array(motorSim.vxa)
    vxb = pylab.array(motorSim.vxb)    
    vxc = pylab.array(motorSim.vxc)
    cmd_d = pylab.array(foc.cmd_d)
    cmd_q = pylab.array(foc.cmd_q)
    id = pylab.array(foc.measured_id)
    iq = pylab.array(foc.measured_iq)

    pwm_t = pylab.array(pwm_t) * 1e3
    velocity = pylab.array(motorSim.velocity)
    position = pylab.array(motorSim.position)


    torque   = pylab.array(motorSim.torque)


    if True:
        pylab.figure()
        pylab.subplot(3,1,1)
        
        pylab.plot(ts,1e3*torque,'k')
        pylab.plot(tc,1e3*pylab.array(foc.target_torque),'r')
        #pylab.plot(ts,ib,'k')
        #pylab.plot(ts,ic,'g')
        #pylab.ylim(-30,30)
        pylab.subplot(3,1,2)
        pylab.plot(ts,position)
        #pylab.plot(tc,1e3*pylab.array(foc.measured_iq),'k',label='measured iq')
        #pylab.ylim(-50,30)
        #pylab.plot(tc, foc.target_iq, 'r', label='target Iq')
        #pylab.plot(tc, foc.measured_iq, 'g', label='measured Iq')
        pylab.subplot(3,1,3)
        pylab.plot(ts,backemf_a)
        pylab.show()









    
true = 1
if true:
    simulation()

         



        

        
        
        























        


        
    

