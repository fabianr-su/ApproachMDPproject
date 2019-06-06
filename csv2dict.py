
""" Read flight data from a csv to python dictionary.
"""

import csv, os, glob, collections
import matplotlib.pyplot as plt
"""
Class: csvImport
----------------
Read csv files from a folder and save the data to into a dictionary.
Given a folder path as string, the class can read all csv files within that folder. Expects
data within the csvs in the form time, distance to end of flight, velocity, latitude, 
longitude, altitude (in ft).
The returned dictionary is of the form:
{ICAOid1: {'time': [], 'lat': [], 'lon': [], 'alt': [], 'vel': [], 'distToEnd': []}, \
 ICAOid2: {'time': [], 'lat': [], 'lon': [], 'alt': [], 'vel': [], 'distToEnd': []}, ...}

Written by Fabian Rothmaier

@param:
folder: <string> of folder path

@return:
acDict: <dict<string: dict<string: list<float>>>>
"""
class csvImport():
    
    # Function: Init
    # --------------
    # Constructor that initializes a csvImport object which has a folder 
    # from which it can read and a dictionary to which it can save data.
    def __init__(self, folder):
        self.folder = folder
        self.acDict = {}

    # Function: readcsvs
    # --------------
    # Reads data from all csv files in the given folder and saves them to the dictionary.
    def readcsvs(self):
        allFiles = glob.glob(os.path.join(self.folder, '*.csv'))  # find all the csv files
        for file in allFiles:
            with open(file, 'rb') as f:
                reader = csv.reader(f, delimiter=',')
                date = file[-17:-11]
                ICAOid = file[-11:-5]
                segment = int(file[-5])
                time, dToEnd, vel, lat, lon, alt = [], [], [], [], [], []   # list for a single segment
                
                for row in reader:
                    # get the elements of the row, add to list
                    time.append(float(row[0]))
                    lat.append(float(row[1]))
                    lon.append(float(row[2]))
                    alt.append(float(row[3]) / 3.281) # ft to m conversion
                    vel.append(float(row[4]) * 0.51) # kt to m/s
                    dToEnd.append(float(row[5]) / 1000) # m to km conversion
            
            self.acDict[(date, ICAOid, segment)] = {'time': time, 'lat': lat, 'lon': lon, \
                                                  'alt': alt, 'vel': vel, 'distToEnd': dToEnd}


    def plotVerticalProfiles(self, fs=18, fSize=(20, 10), minAlt=1000):
        f = plt.figure(figsize=fSize)
        plt.grid()
        for flight in self.acDict.keys():
            if self.acDict[flight]['alt'][-1] < minAlt:
                plt.plot(self.acDict[flight]['distToEnd'], \
                         [alt - self.acDict[flight]['alt'][-1] \
                          for alt in self.acDict[flight]['alt']], 'b')
        plt.xlabel('Distance to Threshold in km', fontsize=fs)
        plt.ylabel('Altitude in m', fontsize=fs)
        return f

    def plotVelocityProfiles(self, fs=18, fSize=(20, 10)):
        f = plt.figure(figsize=fSize)
        plt.grid()
        for flight in self.acDict:
            velocity = self.acDict[flight]['vel']
            plt.plot([self.acDict[flight]['distToEnd'][i] \
                      for i, vel in enumerate(velocity) \
                      if i == 0 or vel < velocity[i-1]+1], \
                     [vel \
                      for i, vel in enumerate(velocity) \
                      if i == 0 or vel < velocity[i-1]+1], 'b')
        plt.xlabel('Distance to Threshold in km', fontsize=fs)
        plt.ylabel('Velocity in m/s', fontsize=fs)
        return f



# d = readcsvs('adsb-data/logs/')


                
