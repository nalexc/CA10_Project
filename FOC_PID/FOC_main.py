from math import sin, cos, sqrt, pi
#from control import *
#from control.matlab import *
#import matplotlib.pyplot as plt
from PID_RW_FOC import PID

        


class FOC:
    def __init__(self,dt,motor,d_gains,q_gains):

        self.dt = dt
        self.motor = motor
        self.d_gains = d_gains # dirrect axis gain
        self.q_gains = q_gains # torque axis gain
        
        self.ctrl_d = PID(d_gains) # takes two arguments Kp,Ki
        self.ctrl_q = PID(q_gains) # takes two arguments Kp,Ki

        # measurements
        
        self.measured_iq = [0.0]
        self.measured_id = [0.0]
        self.measured_torque = [0.0]

        self.t = [0.0]

        # Targets
         
        self.target_id = [0.0]  # the flux current id should be referenced to 0 since there it gives the highest eficiency
        self.target_iq = [0.0]  # target i_q is the targetTorque/Kt
        self.target_torque = [0.0] # the requested torque from , if any, previews steps 

        # output comands
        self.cmd_a = [0.0]  # FOC outputs
        self.cmd_b = [0.0]
        self.cmd_c = [0.0]

        self.cmd_q = [0.0]   # output from the q axis controller
        self.cmd_d = [0.0]   # outup from the d axis controller
        

        # estimate the electrical angle offset

       # self.est_el_angle_offset = [0.0] # estimation from the back emf 
       # self.el_angle_offset = [0.0] # ideally is pi/6 from hall sensor
        
        
##    def reset(self, velocity, supply):
##        self.ctrl_d.I_term[-1] = 0.0        
##        self.ctrl_q.I_term[-1] = velocity * self.motor.torque_constant* 0.7404804896930609 / supply * pi/3.0
        
        

    def update(self, t, target_torque, position,velocity, ia, ib, ic, supply):
        motor = self.motor
        dt = self.dt
        self.target_torque.append(target_torque)
        #############################################################
        #############################################################
        ####################conversions
        torque_constant = motor.torque_constant
##        torque_limit = 0.0
##        if self.current_limit > 0.0:
##            cmd_q_filt = self.cmd_q_filt[-1]
##            if abs(cmd_q_filt) > 1e-3:
##                sqrt2 = 1.4142135623730951 # sqrt(2)
##                torque_limit = -torque_constant*self.current_limit*sqrt2/cmd_q_filt
## #                    target_torque = min(target_torque, torque_limit)
##            else:
##                    target_torque = max(target_torque, torque_limit)
            
        if target_torque > motor.maxTorque:
            target_torque = motor.maxTorque
        elif target_torque < -motor.maxTorque:
            target_torque = - motor.maxTorque
             
           
           

           
        # clark transform
        # translates the three axis currents into two orthogonal time varying axis 
        
        # as init axis 0.0 I use the hall sensor assuming that is appart
        # from A axis 30 degrees

        root23 = 0.816496580927726  # sqrt(2/3)
        s30 =  0.49999999999999994  # sin(pi/6)
        c30 =  0.8660254037844387  # cos(pi/6)

        i_alpha = root23 * (ia  - s30*ic - s30*ib)
        i_beta =  root23 * (c30*ic - c30*ib)


        # electrical angle = P/2 * mechanical angle were P is the number of poles in our case 8 poles
        # or theta_el = Np * theta_m were Np is the number of pole pairs

        Np = self.motor.pole_pairs


        theta_el = position * Np  - self.el_angle_offset[-1]    # how much phase degree has the motor between back EMF and index pulse
                                                     # it should be estimated from back emf estimator  


        # park transform

        s_el = sin(theta_el)
        c_el = cos(theta_el)

        measured_id = c_el * i_alpha + s_el * i_beta
        measured_iq = -s_el * i_alpha + c_el *i_beta


        Torque_constant = self.motor.torque_constant
        target_id = 0.0
        target_iq = target_torque / Torque_constant # maybe it needs a minus depending on the feedback sign
        

        cmd_q = self.ctrl_d.update(t,dt,(target_id - measured_id))
        cmd_d = self.ctrl_q.update(t,dt,(target_iq - measured_iq))

        # I have to normalize because we care about the direction of the vector (90 degrees appart)

        mag = sqrt(cmd_d*cmd_d + cmd_q*cmd_q)
        if mag > 1.0:
            cmd_q /= mag
            cmd_d /= mag

        # inverse clarke

        cmd_alpha = cmd_d * c_el - cmd_q * s_el
        cmd_beta = cmd_d * s_el + cmd_q * c_el
   

        # inverse park  for star connection V_coil = 1/sqrt(3) * V_line
        inv_sqrt3 = 0.5773502691896258 # 1/sqrt(3)
        cmd_a = inv_sqrt3 * ( cmd_alpha )
        cmd_b = inv_sqrt3 * (cmd_beta*c30 - cmd_alpha*s30)
        cmd_c = inv_sqrt3 * (-cmd_beta*c30 - cmd_alpha*s30)
         

        el_velocity = velocity * self.motor.pole_pairs

        # Determine the difference between the commanded value of direct voltage 
        # and what is should be based on velocity and current


        self.error_vd = [0.0]
        self.est_vd = [0.0]
        self.cmd_vd = [0.0]
        self.backemf_voltage = [0.0]






        
##        cmd_vd = supply*self.cmd_d[-1] 
##        L = self.motor.L / sqrt(2)
##        est_vd = el_velocity * measured_id * L 
##        error_vd = cmd_vd - est_vd
##        backemf_constant = self.motor.torque_constant
##        backemf_voltage = velocity * backemf_constant
##        if abs(backemf_voltage) > 1.0:
##            est_el_angle_offset = -error_vd / backemf_voltage
##            alpha = 0.01
##            el_angle_offset = self.el_angle_offset[-1] + alpha * est_el_angle_offset 
##        else:
##            est_el_angle_offset = 0.0
##            el_angle_offset = self.el_angle_offset[-1]
##
         
        

        # update values
        self.t.append(t)
        self.measured_id.append(measured_id)
        self.measured_iq.append(measured_iq)
        self.target_id.append(target_id)
        self.target_iq.append(target_iq)
        self.cmd_q.append(cmd_q)
        self.cmd_d.append(cmd_d)
        self.cmd_a.append(cmd_a)
        self.cmd_b.append(cmd_b)
        self.cmd_c.append(cmd_c)
        self.target_torque.append(target_torque)
##        self.est_el_angle_offset.append(est_el_angle_offset)
##        self.el_angle_offset.append(el_angle_offset)
##        self.est_vd.append(est_vd)
##        self.error_vd.append(error_vd)
##        self.cmd_vd.append(cmd_vd)
##        self.backemf_voltage.append(backemf_voltage)
##
##
        return (cmd_a, cmd_b, cmd_c)
  

         
        

        



        
            











        

        

        
    
        
        


        
















        
