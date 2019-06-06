import csv2dict, pickle
import plotUtil

from projectUtil import *


## instantiate atmosphere, aircraft classes
atmosphere = ISA()

# aircraft class: (mass [kg], prop. efficiency, idleFuelCons [kg/s], minSpeeds [m/s], maxSpeeds [m/s])
b737 = aircraft(55000., 0.33, 600./3600, [90, 80, 70, 65], [150, 115, 100, 90])
b747 = aircraft(350000., 0.33, 1500./3600, [100, 90, 80, 70], [150, 130, 110, 90])

# approachMDP class: (atmosphere, aircraft, initialState, finalState)
initState = (11000, 105, 0, 250000)
FAFState = (1000, 70, 3)
mdp737 = ApproachMDP(atmosphere, b737, initState, FAFState)

## import real aircraft trajectories

csvDict = csv2dict.csvImport('adsb-data/logs') # generate the class
try:
    csvDict.readcsvs() # actually import the data
except:
	pass

## load computed policy:
with open('policy737cruise1000vi', 'rb') as f:
# with open('policy737cruise1000vi', 'rb') as f:
    vi737 = pickle.load(f)


## To experiment: Where does the approach start?
altitude = 11000  # altitude in meter (1 ft = 0.3048 m) [choose integer between 1000 and 10000]
speed = 105      # speed in m / s (1 mph = 0.45 m / s) [choose integer between 70 and 110]
config = 0       # flap setting [choose between 0 and 3]
distance = 250   # distance from the FAF in km (1 mile = 1.609 km) [choose even integer between 50 and 200]
# plot results!(altitude, speed, config, distance*1000)
plotUtil.plotTrajectory(mdp737, vi737, csvDict, (altitude, speed, config, distance*1000), figSize=(20,8))

