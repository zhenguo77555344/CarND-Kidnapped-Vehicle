#-*- coding:utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import csv
import time


class logParase:
    def __init__(self,simulatorLog,pfIniLog):
        print("Hello logParase")
        self.simulatorLog = simulatorLog
        self.pfIniLog = pfIniLog

        self.simulatorBuffSize = 1
        self.pfIniBuffSize = 20

        self.simulatorIni = {'timestamp':[0]*self.simulatorBuffSize,'sense_x':[0]*self.simulatorBuffSize,'sense_y':[0]*self.simulatorBuffSize,'sense_theta':[0]*self.simulatorBuffSize}
        self.pfIni = {'timestamp':[0]*self.pfIniBuffSize,'p_x':[0]*self.pfIniBuffSize,'p_y':[0]*self.pfIniBuffSize,'p_theta':[0]*self.pfIniBuffSize}
        #cout <<"PF@init"<<","<<p.id<<","<< p.x<<","<<p.y<<","<<p.theta<<endl;

    def pfInitial(self):
        csv_file = csv.reader(open('./log/PF_Initial.csv'))
        #print(csv_file)
        # val is value of line in csv_file
        for val in csv_file:
            print(val)
            print("type of val",type(val))
            #print("length of val",len(val))
            #print("val[0]",val[0])
            #print("val[1]",val[1])
            if val[0] == "SIMU@":
                '''
                print("SIMU@",111111111111)
                self.simulatorLog.append(val[0])
                self.simulatorLog.append(val[1])
                self.simulatorLog.append(val[2])
                self.simulatorLog.append(val[3])
                '''
                dataSimu=list(map(float,val[1:5]))
                print('SImu data',dataSimu)
                self.setsimulatorIniBuffer(dataSimu)
            
            if val[0] == "PF@init":
                '''
                print("PF@init: ",22222222222)
                self.pfIniLog.append(val[0])
                self.pfIniLog.append(val[1])
                self.pfIniLog.append(val[2])
                self.pfIniLog.append(val[3])
                self.pfIniLog.append(val[4])
                '''
                dataPFIni=list(map(float,val[1:6]))
                print('PF@init data',dataPFIni)
                self.setPFIniBuffer(dataPFIni)
    def setsimulatorIniBuffer(self,data):
        self.simulatorIni['timestamp'].pop(0)
        self.simulatorIni['sense_x'].pop(0)
        self.simulatorIni['sense_y'].pop(0)
        self.simulatorIni['sense_theta'].pop(0)

        self.simulatorIni['timestamp'].append(0)
        self.simulatorIni['sense_x'].append(data[1])
        self.simulatorIni['sense_y'].append(data[2])
        self.simulatorIni['sense_theta'].append(data[3])

    def getSimulatorIniBuffer(self):
        return self.simulatorIni

    def setPFIniBuffer(self,data):
        self.pfIni['timestamp'].pop(0)
        self.pfIni['p_x'].pop(0)
        self.pfIni['p_y'].pop(0)
        self.pfIni['p_theta'].pop(0)

        self.pfIni['timestamp'].append(data[0])
        self.pfIni['p_x'].append(data[1])
        self.pfIni['p_y'].append(data[2])
        self.pfIni['p_theta'].append(data[3])

    def getPFIniBuffer(self):
        return self.pfIni

    def showData(self):
        SimuIniData = self.getSimulatorIniBuffer()
        PFIniData = self.getPFIniBuffer()
        print('SimuIniData:---->',SimuIniData)
        print('PFIniData---->',PFIniData)

        sense_x = SimuIniData['sense_x']
        sense_y = SimuIniData['sense_y']

        #print('type of PFIniData['p_x']',type(PFIniData['p_x']))
        #p_x = list(np.around(PFIniData['p_x'], decimals=1))
        #p_y = list(np.around(PFIniData['p_y'], decimals=1))

        p_x = PFIniData['p_x']
        p_y = PFIniData['p_y']

        print("type of p_x",type(p_x))
        print("p_x---->",p_x)
        #plt.figure('name','Particle Filter Initial Data')
        #plt.plot(p_x,p_y,label="Particle Filter Initial Data")
        plt.title("Particle Filter Initial Data")
        plt.scatter(np.round(p_x,decimals = 1), np.round(p_y,decimals=1), s=20, c="blue", marker='1')
        #plt.scatter(p_x, p_y, s=20, c="blue", marker='1')
        plt.scatter(sense_x,sense_y,s=20, c="red", marker='o')
        plt.xlabel('p_x')
        plt.ylabel('p_y')
        plt.show()

    def ptf(self):
        print (time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
        print("self.simulatorLog->",self.simulatorLog)
        print("self.pfIniLog->",self.pfIniLog)
        print("self.simulatorIni",self.simulatorIni)
        print("getSimulatorIniBuffer",self.getSimulatorIniBuffer())
        print("getPFIniBuffer",self.getPFIniBuffer())


def main():
    print("Hello Vscode!\n")
    simulatorLog = []
    pfIniLog = []
    log = logParase(simulatorLog,pfIniLog)
    log.pfInitial()
    #log.ptf() 
    log.showData()


if __name__ == "__main__":
    main()





