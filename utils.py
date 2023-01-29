import sys
sys.path.insert(0, '/usr/share/sumo/tools/output')
from statisticsElements import Assign, VehInformationReader
from xml.sax import make_parser

def getBasicStats(method):

    vehicles = []
    parser = make_parser()
    parser.setContentHandler(VehInformationReader(vehicles))
    parser.parse(method)

    totalVeh = 0.
    totalTravelTime = 0.
    totalTravelLength = 0.
    totalTravelSpeed = 0.
    totalWaitTime = 0.
    totalDiffSpeed = 0.
    totalDiffLength = 0.
    totalDiffWaitTime = 0.
    totalDiffTravelTime = 0.
    totalDepartDelay = 0.

    for veh in vehicles:
        totalVeh += 1
        veh.method = method
        # unit: speed - m/s; traveltime - s; travel length - m
        veh.speed = veh.travellength / veh.traveltime
        totalTravelTime += veh.traveltime
        totalTravelLength += veh.travellength
        totalWaitTime += veh.waittime
        totalTravelSpeed += veh.speed
        totalDepartDelay += veh.departdelay

    totalVehDivisor = max(1, totalVeh)  # avoid division by 0
    avgTravelTime = totalTravelTime / totalVehDivisor
    avgTravelLength = totalTravelLength / totalVehDivisor
    avgTravelSpeed = totalTravelSpeed / totalVehDivisor
    avgWaitTime = totalWaitTime / totalVehDivisor
    avgDepartDelay = totalDepartDelay / totalVehDivisor
    for veh in vehicles:
        totalDiffTravelTime += (veh.traveltime - avgTravelTime)**2
        totalDiffSpeed += (veh.speed - avgTravelSpeed)**2
        totalDiffLength += (veh.travellength - avgTravelLength)**2
        totalDiffWaitTime += (veh.waittime - avgWaitTime)**2

    # SD: standard deviation
    SDTravelTime = (totalDiffTravelTime / totalVehDivisor)**(0.5)
    SDLength = (totalDiffLength / totalVehDivisor)**(0.5)
    SDSpeed = (totalDiffSpeed / totalVehDivisor)**(0.5)
    SDWaitTime = (totalDiffWaitTime / totalVehDivisor)**(0.5)

    return Assign(method, totalVeh, totalTravelTime, totalTravelLength,
                                 totalDepartDelay, totalWaitTime, avgTravelTime,
                                 avgTravelLength, avgTravelSpeed, avgDepartDelay,
                                 avgWaitTime, SDTravelTime, SDLength, SDSpeed, SDWaitTime)