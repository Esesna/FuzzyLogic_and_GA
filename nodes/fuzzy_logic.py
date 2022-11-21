import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from functools import reduce
import time
import matplotlib.pyplot as plt

class FuzzyLogic():
    def __init__(self):
        self.n = 5 #count bit
        self.countParam = 8
        self.setParametrs(
            [
                1,1,1,1,1,
                1,1,1,1,1,
                1,1,1,1,1,
                1,1,1,1,1,
                1,1,1,1,1,
                1,1,1,1,1,
                1,1,1,1,1,
                1,1,1,1,1,
            ]
            )
        self.initTerm()
        pass

    def reInit(self, listBit):
        self.setParametrs(listBit)
        self.initTerm()


    def setParametrs(self, listBit):
        n = self.n
        cp = self.countParam

        listByte = np.zeros(cp)

        for i in range(cp):
            listByte[i] = 0

            for j in range(n):
                listByte[i] += listBit[j + i * n] * (2 ** j)

        self.paramForFL = np.zeros(cp)
        limitList = [
            # range front
            [0.1, 0.5],
            [0.5, 1.0],
            [1.0, 1.5],
            [1.5, 2.0],
            # range flank
            [0.1, 0.5],
            [0.5, 1.0],
            [1.0, 1.5],
            [1.5, 2.0],
        ]
        for i in range(cp):
            self.paramForFL[i] = self.trForFL(listByte[i], limitList[i][0], limitList[i][1])
            # print(i, " : ", self.paramForFL[i])
        

    def bitToInt(self, listBit):
        return 0

    def trForFL(self, d, xmin, xmax):
        n = self.n
        out = xmin + d / (2**n - 1) * (xmax - xmin)
        return out

    def trForGA(self, x, xmin, xmax):
        n = self.n
        out = (x - xmin)*(2**n - 1) / (xmax - xmin)
        return out

    def my_or(self, val1, val2):
        return val1 | val2

    def my_and(self, vals):
        val = vals[0]
        for i in range(1,len(vals)):
            val = val & vals[i]
        return val

    def my_no(self, val1):
        return ~ val1

    def get_keys(self, dict_, key):
        return [i[key] for i in dict_.values()]

    def initOAFLC(self):
        range_front = ctrl.Antecedent(np.arange(0.1, 2, 0.05), "front")
        range_front['near'] =       fuzz.trapmf(range_front.universe, [               0.1,                0.1, self.paramForFL[0], self.paramForFL[1]])
        range_front['middle'] =     fuzz.trapmf(range_front.universe, [self.paramForFL[0], self.paramForFL[1], self.paramForFL[2], self.paramForFL[3]])
        range_front['far'] =        fuzz.trapmf(range_front.universe, [self.paramForFL[2], self.paramForFL[3],                  2,                  2])
        # range_front.view()

        range_left = ctrl.Antecedent(np.arange(0.1, 2, 0.05), "left")
        range_left['near'] =       fuzz.trapmf(range_left.universe, [               0.1,                0.1, self.paramForFL[4], self.paramForFL[5]])
        range_left['middle'] =     fuzz.trapmf(range_left.universe, [self.paramForFL[4], self.paramForFL[5], self.paramForFL[6], self.paramForFL[7]])
        range_left['far'] =        fuzz.trapmf(range_left.universe, [self.paramForFL[6], self.paramForFL[7],                  2,                  2])

        range_right = ctrl.Antecedent(np.arange(0.1, 2, 0.05), "right")
        range_right['near'] =       fuzz.trapmf(range_right.universe, [               0.1,                0.1, self.paramForFL[4], self.paramForFL[5]])
        range_right['middle'] =     fuzz.trapmf(range_right.universe, [self.paramForFL[4], self.paramForFL[5], self.paramForFL[6], self.paramForFL[7]])
        range_right['far'] =        fuzz.trapmf(range_right.universe, [self.paramForFL[6], self.paramForFL[7],                  2,                  2])

        rules = [
            ctrl.Rule(range_front['far']                                                            , [self.vel['Full Velocity'], self.rot['Front'] ]),
            ctrl.Rule(range_front['middle'] & range_left['far']                                     , [self.vel['Slow Velocity'], self.rot['Left Small'] ]),
            ctrl.Rule(range_front['near'] & range_left['far']                                       , [self.vel['Zero Velocity'], self.rot['Left Big'] ]),
            
            ctrl.Rule(range_front['middle'] & self.my_no(range_left['far']) & range_right['far']    , [self.vel['Slow Velocity'], self.rot['Right Small'] ]),
            ctrl.Rule(range_front['near'] & self.my_no(range_left['far']) & range_right['far']      , [self.vel['Zero Velocity'], self.rot['Right Big'] ]),

            ctrl.Rule(range_front['middle'] & range_left['middle'] & self.my_no(range_right['far']) , [self.vel['Slow Velocity'], self.rot['Front'] ]),
            ctrl.Rule(range_front['near'] & range_left['middle'] & self.my_no(range_right['far'])   , [self.vel['Zero Velocity'], self.rot['Left Big'] ]),

            ctrl.Rule(range_front['middle'] & range_left['near'] & range_right['middle']            , [self.vel['Slow Velocity'], self.rot['Front'] ]),
            ctrl.Rule(range_front['near'] & range_left['near'] & range_right['middle']              , [self.vel['Zero Velocity'], self.rot['Right Big'] ]),

            ctrl.Rule(range_front['middle'] & range_left['near'] & range_right['near']              , [self.vel['Slow Velocity'], self.rot['Front'] ]),
            ctrl.Rule(range_front['near'] & range_left['near'] & range_right['near']                , [self.vel['Zero Velocity'], self.rot['Left Big'] ]),
        ]

        x_ctrl = ctrl.ControlSystem(rules)
        self.OAFLC = ctrl.ControlSystemSimulation(x_ctrl)

    def initTFLC(self):
        error_in_angle = ctrl.Antecedent(np.arange(-180, 180, 3), "ErrorInAngle")
        error_in_angle['N'] =          fuzz.trapmf(error_in_angle.universe, [-180,  -180,  -90, -45])
        error_in_angle['SN'] =         fuzz.trapmf(error_in_angle.universe, [ -90,  -45,  -45,   0])
        error_in_angle['Z'] =          fuzz.trapmf(error_in_angle.universe, [ -45,    0,    0,  45])
        error_in_angle['SP'] =         fuzz.trapmf(error_in_angle.universe, [   0,   45,   45,  90])
        error_in_angle['P'] =          fuzz.trapmf(error_in_angle.universe, [  45,   90,   180, 180])

        distance = ctrl.Antecedent(np.arange(0, 2, 0.05), "distance")
        distance['near'] =       fuzz.trapmf(distance.universe, [0, 0, 0, 0.5])
        distance['far'] =        fuzz.trapmf(distance.universe, [ 0,  0.5,  2,   2])

        rules = [
            ctrl.Rule(distance['near']                      , [self.vel['Zero Velocity'], self.rot['Front'] ]),
            ctrl.Rule(error_in_angle['N'] & distance['far'] , [self.vel['Zero Velocity'], self.rot['Left Big'] ]),
            ctrl.Rule(error_in_angle['SN'] & distance['far'], [self.vel['Slow Velocity'], self.rot['Left Small'] ]),
            ctrl.Rule(error_in_angle['Z'] & distance['far'] , [self.vel['Full Velocity'], self.rot['Front'] ]),
            ctrl.Rule(error_in_angle['SP'] & distance['far'], [self.vel['Slow Velocity'], self.rot['Right Small'] ]),
            ctrl.Rule(error_in_angle['P'] & distance['far'] , [self.vel['Zero Velocity'], self.rot['Right Big'] ]),
        ]

        x_ctrl = ctrl.ControlSystem(rules)
        self.TFLC = ctrl.ControlSystemSimulation(x_ctrl)

    def initTerm(self):
        self.rot = ctrl.Consequent(np.arange(-0.5, 0.5, 0.01), "rotation")
        self.rot['Left Big'] =       fuzz.trapmf(self.rot.universe, [-0.5, -0.5, -0.4, -0.3])
        self.rot['Left Small'] =     fuzz.trapmf(self.rot.universe, [-0.4, -0.2, -0.1, -0.0])
        self.rot['Front'] =          fuzz.trapmf(self.rot.universe, [-0.1,    0,    0,  0.1])
        self.rot['Right Big'] =      fuzz.trapmf(self.rot.universe, [ 0.0,  0.1,  0.2,  0.4])
        self.rot['Right Small'] =    fuzz.trapmf(self.rot.universe, [ 0.3,  0.4,  0.5,  0.5])

        self.vel = ctrl.Consequent(np.arange(-0.02, 0.22, 0.01), "velocity")
        self.vel['Zero Velocity'] =     fuzz.trapmf(self.vel.universe, [-0.02, 0, 0.0, 0.02])
        self.vel['Slow Velocity'] =     fuzz.trapmf(self.vel.universe, [0.01, 0.05, 0.11, 0.15])
        self.vel['Full Velocity'] =     fuzz.trapmf(self.vel.universe, [0.13, 0.15, 0.22, 0.22])
        
        # self.vel = ctrl.Consequent(np.arange(-0.02, 0.5, 0.001), "velocity")
        # self.vel['Zero Velocity'] =     fuzz.trapmf(self.vel.universe, [-0.02, 0, 0.0, 0.02])
        # self.vel['Slow Velocity'] =     fuzz.trapmf(self.vel.universe, [0.00, 0.1, 0.15, 0.3])
        # self.vel['Full Velocity'] =     fuzz.trapmf(self.vel.universe, [0.15, 0.3, 0.5, 0.5])

        self.initTFLC()
        self.initOAFLC()

    def computeOAFLC(self, range_left, range_front, range_right):
        self.OAFLC.input['front'] = range_front
        self.OAFLC.input['left'] = range_left
        self.OAFLC.input['right'] = range_right
        self.OAFLC.compute()
        return self.OAFLC.output['velocity'], self.OAFLC.output['rotation']

    def computeTFLC(self, ErrorInAngle, distance):
        self.TFLC.input['ErrorInAngle'] = ErrorInAngle
        self.TFLC.input['distance'] = distance
        self.TFLC.compute()
        return self.TFLC.output['velocity'], self.TFLC.output['rotation']

    def view(self):
        # print("vel: ", self.xx.output['velocity'])
        # print("rot: ", self.xx.output['rotation'])
        self.vel.view(sim = self.xx)
        self.rot.view(sim = self.xx)