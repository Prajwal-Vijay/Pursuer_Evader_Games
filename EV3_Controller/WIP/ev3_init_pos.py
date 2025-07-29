from numpy import linalg.norm as norm
import gogoal
import ev3_gotogoal

class initialiser(self):

    def __init__(self,p_pos,e_pos,p_ini,e_ini):
        self.p0 = p_pos[0] # Position of pursuer 0
        self.p1 = p_pos[1] # Position of pursuer 1
        self.e = e_pos # Position of evader
        self.p0i = p_ini[0]
        self.p1i = p_ini[1]
        self.ei = e_ini
        self.tol = 0.08
        self.p0s=0
        self.p1s=0
        self.es=0

    def send_vel(self):
       
       
    def set(self,bot):
        if bot == "p0":
            if (self.p0s == 0 and abs(norm(self.p0 - self.p0i))>self.tolerance):
                p0_move = gogoal.OmniRobot(np.array([self.p0i[0],self.p0i[1]],0))
            else:
                p0_move = [0,0,0]
                self.p1s=1
        elif bot == "p1":
            if (self.p1s == 0 and abs(norm(self.p1 - self.p1i))>self.tolerance):
                p1_move = gogoal.OmniRobot(np.array([self.p1i[0],self.p1i[1]],0))
            else:
                p1_move = [0,0,0]
                self.p1s=1
        elif bot == "e":
            if (self.es == 0 and abs(norm(self.e - self.ei))>self.tolerance):
                e_move = gogoal.OmniRobot(np.array([self.ei[0],self.ei[1]],0))
            else:
                e_move = [0,0,0]
                self.p1s=1


    def set_pos(self):
        self.set()
        if (self.p0s == 0 or self.p1s == 0 or self.es == 0):
            while (self.p0s == 0):
                send_wheel_velocities(p0_move)

def main():
    

if __name__ == "__main__":
    main()
