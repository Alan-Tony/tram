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

def get_junction_info():
        neighbor_map = {}
        # corner nodes
        neighbor_map['nt1'] = ['nt6', 'nt2', 'np1', 'np20']
        neighbor_map['nt5'] = ['nt10', 'np6', 'np5', 'nt4']
        neighbor_map['nt21'] = ['np15', 'nt22', 'nt16', 'np16']
        neighbor_map['nt25'] = ['np11', 'np10', 'nt20', 'nt24']
        # edge nodes
        neighbor_map['nt2'] = ['nt7', 'nt3', 'np2', 'nt1']
        neighbor_map['nt3'] = ['nt8', 'nt4', 'np3', 'nt2']
        neighbor_map['nt4'] = ['nt9', 'nt5', 'np4','nt3']
        neighbor_map['nt22'] = ['np14', 'nt23', 'nt17', 'nt21']
        neighbor_map['nt23'] = ['np13', 'nt24', 'nt18', 'nt22']
        neighbor_map['nt24'] = ['np12', 'nt25', 'nt19', 'nt23']
        neighbor_map['nt10'] = ['nt15', 'np7', 'nt5', 'nt9']
        neighbor_map['nt15'] = ['nt20', 'np8', 'nt10', 'nt14']
        neighbor_map['nt20'] = ['nt25', 'np9', 'nt15', 'nt19']
        neighbor_map['nt6'] = ['nt11', 'nt7', 'nt1', 'np19']
        neighbor_map['nt11'] = ['nt16', 'nt12', 'nt6', 'np18']
        neighbor_map['nt16'] = ['nt21', 'nt17', 'nt11', 'np17']
        # internal nodes
        for i in [7, 8, 9, 12, 13, 14, 17, 18, 19]:
            n_node = 'nt' + str(i + 5)
            s_node = 'nt' + str(i - 5)
            w_node = 'nt' + str(i - 1)
            e_node = 'nt' + str(i + 1)
            cur_node = 'nt' + str(i)
            neighbor_map[cur_node] = [n_node, e_node, s_node, w_node]

        controlled_lanes = {}
        for i in range(25):
             
            # Order: north, south, east-left-lane, west-left-lane, east-right-lane, west-right-lane,
            # where the direction is the lanes direction relative to the traffic light
            tlsID = 'nt'+str(i+1)
            [n, e, s, w] = neighbor_map[tlsID]
            controlled_lanes[tlsID] = [
                n + '_' + tlsID + '_0',
                s + '_' + tlsID + '_0',
                e + '_' + tlsID + '_1',
                w + '_' + tlsID + '_1',
                e + '_' + tlsID + '_0',
                w + '_' + tlsID + '_0',
            ]

        return neighbor_map, controlled_lanes