import math, collections
from util import *


############################################################

# Environment class.
# Calculate pressure, temperature, and density for a given altitude
# according to the ICAO standard atmosphere (ISA) model.
# Supports altitudes up to 20 000 m; altitude queries expected in m.
class ISA:
    def __init__(self):
        self.p_0   = 101325       # N/m^2 pressure at sea level
        self.T_0   = 15+273.15    # K temperature at sea level
        self.rho_0 = 1.225        # kg/m^3 density at sea level
        self.R     = 8.31432      # Nm/molK
        self.M_g   = 0.0289644    # kg/mol
        self.g     = 9.80665      # m/s^2 gravitational constant
        self.L_b   = -0.0065      # K/m Temperature lapse rate
        self.h_t   = 11000        # m altitude of tropopause
        
    # compute temperature at altitude
    def getT(self, alt):
        # handle list query
        if isinstance(alt, collections.Iterable):
            return [self.getT(a) for a in alt]
        # check if above tropopause
        if alt > self.h_t:
            return self.getT(self.h_t)
        else:
            return self.T_0 + self.L_b * alt
        
    # compute pressure at altitude
    def getP(self, alt):
        # handle list query
        if isinstance(alt, collections.Iterable):
            return [self.getP(a) for a in alt]
        # check if above tropopause
        if alt > self.h_t:
            p_t = self.getP(self.h_t)
            return p_t * math.exp(-self.g * self.M_g * (alt - self.h_t) / self.R / self.getT(alt))
        else:
            return self.p_0 * (self.T_0 / (self.T_0 + self.L_b*alt)) ** (self.g*self.M_g/self.R/self.L_b)
    
    # compute density at altitude
    def getRho(self, alt):
        # handle list query
        if isinstance(alt, collections.Iterable):
            return [self.getRho(a) for a in alt]
        # check if above tropopause
        if alt > self.h_t:
            rho_t = self.getRho(self.h_t)
            return rho_t * math.exp(-self.g * self.M_g * (alt - self.h_t) / self.R / self.getT(alt))
        else:
            return self.rho_0 * (1 - self.L_b * alt / self.T_0) ** (1 + self.g*self.M_g/self.R/self.L_b)

############################################################

# Aircraft class. An instance of this class is the input to the approach MDP.
# An aircraft is defined by its weight (constant, in N); 
# its thrust specific fuel consumption (tsfc, in kg / N / s), the fuel used per thrust per time; 
# its lift to drag ratio depending on configuration (unitless)
class aircraft:
    def __init__(self, mass, eta, idleBurnRate, minSpeed, maxSpeed):
        self.mass         = mass # mass in kg
        self.eta          = eta  # net engine efficiency
        self.idleBurnRate = idleBurnRate # kg / s fuel burn at idle thrust
        self.minSpeed     = minSpeed   # list of minimum speeds in m/s depending on configuration
        self.maxSpeed     = maxSpeed   # list of maximum speeds in m/s depending on configuration
        self.b            = 35. # m wing span (of 737-800)
        self.AR           = 9.45; # aspect ratio (of 737-800)
        self.cDp          = [0.0165, 0.018, 0.02, 0.023, 0.03] # parasite drag coeff (of 737-800)
        self.e            = 0.8 #oswald efficiency / span efficiency (of 737-800)
        
    # Get lift coefficient for level flight
    def getcL(self, state, environment):
        speed = state[1]
        return environment.g * self.mass / (0.5 * environment.getRho(0) * speed**2 * (self.b**2 / self.AR))
    
    # Get drag coefficient
    def getcD(self, state, environment):
        speed, config = state[1:3]
        cL = self.getcL(state, environment)
        return self.cDp[config] + cL**2 / (self.e * self.AR * math.pi)
    
    # Get Drag force in N
    def getD(self, state, environment):
        speed = state[1]
        cD = self.getcD(state, environment)
        return 0.5 * environment.getRho(0) * speed**2 * cD * self.b**2 / self.AR


############################################################
# Implementation of the approach MDP class.
class ApproachMDP(MDP):
    def __init__(self, environment, aircraft, initState, FAFstate, dDist=1, verbose=False):
        """
        @param:
        aircraft: instance of the aircraft class to be used in the MDP
        altitude: initial altitude of the aircraft in meter
        speed: initial equivalent airspeed of the aircraft in m/s
        configuration: initial configuration of the aircraft
        distance: initial distance from FAF
        FAFstate: tuple with 3 elements: desired altitude, speed and configuration at the FAF
        """
        self.env      = environment # define environment class
        self.ac       = aircraft # aircraft to be used
        self.altitude, self.speed, self.config, self.distance = initState
        self.FAFstate = FAFstate
        self.kFuel = 43e6       # energy density of fuel: 43 MJ / kg
        self.dD = 1000 * dDist  # distance flown per segment in m
        self.dA = 50 * dDist    # altitude change per climb / descend in m
        self.dS = 5 * dDist     # speed change per accel / decel
        self.verbose = verbose  # print stuff? y/n
    
    # function to get deceleration per distance for idle thrust
    # Note: increases state space too much. Keep at fixed value until better discretization model.
    def getDecel(self, state):
        # alt, v = state[0:2]
        # EdD = self.dD * (self.env.getRho(alt) / self.env.getRho(0))**0.5 # equivalent air distance
        # return int(v - max(v**2 - 2 * self.ac.getD(state, self.env) * EdD / self.ac.mass, \
        #                0)**0.5)
        return self.dS
    
    # function to get descend distance per distance for idle thrust
    # Note: increases state space too much. Keep at fixed value until better discretization model.
    def getDescend(self, state):
        # alt = state[0]
        # EdD = self.dD * (self.env.getRho(alt) / self.env.getRho(0))**0.5 # equivalent air distance
        # return int(self.ac.getD(state, self.env) * EdD / self.ac.mass / self.env.g)
        return self.dA

    # determine the cost (fuel use) depending on aircraft, state and action
    def getCost(self, state, action):
        alt, EAS, config, distance = state
        TAS = EAS * (self.env.getRho(0) / self.env.getRho(alt))**0.5
        maneuverCost, TASafter, altAfter = 0, TAS, alt
        EdD = self.dD * (self.env.getRho(alt) / self.env.getRho(0))**0.5 # equivalent air distance

        # get cost for maneuver
        # if action in ['climb', 'descend', 'accel', 'decel']:
        #     maneuverCost += 1
        # elif action in ['retract', 'extend']:
        #     maneuverCost += 10

        # get altitude after
        if action == 'climb':
            altAfter += self.dA
        elif action == 'descend':
            altAfter -= self.getDescend(state)
        # get speed after
        if action == 'accel':
            TASafter += self.dS
        elif action == 'decel':
            TASafter -= self.getDecel(state)
        
        cLcD = self.ac.getcL(state, self.env) / self.ac.getcD(state, self.env)
        Ereq = self.ac.mass * self.env.g * (altAfter - alt)\
             + EdD * self.ac.getD(state, self.env) \
             + 0.5 * (TASafter**2 - TAS**2) * self.ac.mass
        
        # min fuel burn if idle thrust
        minFuel = self.ac.idleBurnRate * self.dD / TAS
        fuelUsed = max(Ereq / self.ac.eta / self.kFuel, minFuel)
        
        if self.verbose:
            print('Alt: {}, speed: {}, config: {}, dist: {}, action: {}, Fuel required: {} kg'.format( \
                  alt, EAS, config, distance, action, fuelUsed))
        
        return fuelUsed + maneuverCost

    # Return a value of any type capturing the start state of the MDP.
    def startState(self):
        """
        Each state is a tuple with 4 elements:
          -- The first element of the tuple is the altitude.
          -- The second element is the aircraft's speed.
          -- The third element is the aircraft's configuration.
          -- The fourth element is the aircraft's distance from the FAF.
        """
        return (self.altitude, self.speed, self.config, self.distance)

    # Return a list of strings representing actions possible from |state|.
    def actions(self, state):
        """
        Possible actions:
        climb, descend, level flight, extend flaps, retract flaps, accelerate, decelerate.
        Check constraints for each before adding them to the list of possible actions.
        """
        altitude, speed, config, distance = state
        actions = ['climb', 'level']
        if config > 0 and speed >= self.ac.minSpeed[config-1]:
            actions.append('retract')
        if config < len(self.ac.minSpeed)-1 and speed <= self.ac.maxSpeed[config+1]:
            actions.append('extend')
        if altitude > self.getDescend(state):
        	actions.append('descend')
        if speed >= self.ac.minSpeed[config]+self.getDecel(state):
        	actions.append('decel')
        if speed <= self.ac.maxSpeed[config]-self.dS:
        	actions.append('accel')
        
        return actions

    # Given a |state| and |action|, return a list of (newState, prob, reward) tuples
    # corresponding to the states reachable from |state| when taking |action|.
    # Remember that if |state| is an end state, you should return an empty list [].
    def succAndProbReward(self, state, action):
        altitude, speed, config, distance = state
        # check for terminal state
        if state == self.FAFstate + (0,):
            return []
        # check if above/below FAF. Next state will be terminal state, indepent of action.
        if state[3] <= 0:
            cost = 10000 * sum([10000000 * (x - y)**2 for x, y in zip(state[:2], self.FAFstate[:2])]) \
                   + 100000000 * abs(state[2] - self.FAFstate[2])
            return [(self.FAFstate + (0,), 1, -cost)]
        
        # fly approach segment according to chosen action
        reward = -self.getCost(state, action) # independent of probability
        distance -= self.dD
        
        if   action == 'climb':
            altitude -= self.dA
            bestState   = (altitude, speed, config, distance)
            otherState1 = (altitude+10, speed, config, distance)
            otherState2 = (altitude-10, speed, config, distance)
            return [(bestState, 0.8, reward), (otherState1, 0.1, reward), (otherState2, 0.1, reward)]
        elif action == 'descend':
            altitude -= self.getDescend(state)
            bestState   = (altitude, speed, config, distance)
            otherState1 = (altitude+1, speed, config, distance)
            otherState2 = (altitude-1, speed, config, distance)
            return [(bestState, 0.8, reward), (otherState1, 0.1, reward), (otherState2, 0.1, reward)]
        elif action == 'extend':
            bestState   = (altitude, speed, config+1, distance)
            otherState1 = (altitude, speed, config, distance)
            return [(bestState, 0.95, reward), (otherState1, 0.05, reward)]
        elif action == 'retract':
            bestState   = (altitude, speed, config-1, distance)
            otherState1 = (altitude, speed, config, distance)
            return [(bestState, 0.95, reward), (otherState1, 0.05, reward)]
        elif action == 'accel':
            speed += self.dS
            bestState   = (altitude, speed, config, distance)
            otherState1 = (altitude, speed+1, config, distance)
            otherState2 = (altitude, speed-1, config, distance)
            return [(bestState, 0.8, reward), (otherState1, 0.1, reward), (otherState2, 0.1, reward)]
        elif action == 'decel':
            speed -= self.getDecel(state)
            bestState   = (altitude, speed, config, distance)
            otherState1 = (altitude, speed+1, config, distance)
            otherState2 = (altitude, speed-1, config, distance)
            return [(bestState, 0.8, reward), (otherState1, 0.1, reward), (otherState2, 0.1, reward)]
        else:
            return [((altitude, speed, config, distance), 1, reward)]
        
        

    # The discount factor (float or integer) for the MDP.
    def discount(self):
        # Until further work is done: fuel useage "costs" the same anywhere
        # possible idea: make it cost more/less depending on altitude
        return 1

    """
    Things to implement:
    - probabilities
    """