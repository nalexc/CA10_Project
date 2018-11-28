# PID class outputs the controller output signal depending on the error PID.update(t,dt,target-measured)
class PID(object):

    def __init__(self,gains):
        p_gain,i_gain = gains
        self.Pgain = p_gain
        self.Igain = i_gain
        self.ie = [0.0] # integral * dt *Ki  is simple the I term if PI control
        self.t = [0.0] # 
        self.control_out = [0.0]
        
    def update(self,t,dt,error):
        I_term = self.ie[-1]
        I_term += error*self.Igain*dt  # i term   (0.0 + err * Ki)
        if I_term > 1:
            I_term = 1 # limit to 1
        elif I_term < -1:
            I_term = -1  # limit to -1
        control_out = self.Pgain*error + I_term      
        if control_out > 1:
            control_out  = 1    #  unity feedback upper limit (if i was a valve would be 100 percend)
        elif control_out < -1:
            control_out = -1    # lower limit unity feedback
        # store the results
        self.ie.append(I_term)
        return control_out 
