import numpy as np
import time

class AdmittanceController():
    ## Energy tank based variable admittance controller
    def __init__(self, D_a = np.eye(6), M_a = np.eye(6), freq = 100):
        # Predefined params
        self.D_a_ = D_a # damping matrix for admittance control
        self.M_a_ = M_a # mass matrix for admittance control
        self.dt_ = 1.0 / freq

        self.Pd_tilde_ = 0.25 # power dissipation rate
        self.E_thres_ = 10
        self.E_max_ = 50

        # Working params
        self.h_ = 0
        self.E_m = 0
        self.dx_a_ = np.zeros(6) # desired twist admittance
        self.F_ext = np.zeros(6) # external wrench measured by FT sensor
        self.F_h = np.zeros(6)

    def getAdmittanceRatio(self):
        return self.h_

    def getDt(self):
        return self.dt_

    def getRegularedOutput(self, limit):
        self.dx_a_ = np.clip(self.dx_a_, -limit, limit)
        return self.dx_a_

    def computeAdmittanceRatio(self, E_m):
        if E_m > self.E_thres_:
            return (E_m - self.E_thres_) / (self.E_max_ - self.E_thres_)
        return 0 # dead zone

    def computeAdmtOutput(self, F_ext):
        ##  Control Loop
        ddx_a = np.linalg.inv(self.M_a_) @ (-self.D_a_ @ self.dx_a_ + F_ext)

        Pi_tilde = self.dx_a_ @ F_ext
        Po_tilde = self.dx_a_ @ self.F_h
        Pt_tilde = (1 - self.h_) * self.Pd_tilde_

        dE = Pi_tilde - Po_tilde - Pt_tilde
        self.E_m += dE

        # Saturated to [0, Em]
        self.E_m = np.clip(self.E_m, 0, self.E_max_)
        self.h_ = self.computeAdmittanceRatio(self.E_m)
        self.F_h = self.h_ * F_ext 

        self.dx_a_ += ddx_a * self.dt_
        return self.dx_a_

if __name__ == '__main__':
    Da = np.diag([2,2,2,1,1,1])
    Ma = np.diag([2,2,2,2,2,2])
    limit = np.array([0.5, 0.5, 0.5, 0.15, 0.15, 0.15]) # twist limit
    admtCtrl = AdmittanceController(Da, Ma) 
    
    for i in range(100):
        dt = admtCtrl.getDt()
        F_ext = np.zeros(6)
        if (i >30 and i < 50):
            F_ext = np.array([1.5, 1.5, 1.5, 0.2, 0.2, 0.2])
        admtCtrl.computeAdmtOutput(F_ext)
        dxa = admtCtrl.getRegularedOutput(limit)

        print("idx[{}], h: {}\ndxa: {}\n".format(i, admtCtrl.getAdmittanceRatio(), dxa) )
        time.sleep(dt)
    # plot