#!/usr/bin/env python

import os
import sys
import optparse
import numpy as np
from xml.sax import make_parser
sys.path.insert(0, '/usr/share/sumo/tools/output')
import statisticsElements
import utils

# we need to import some python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


from sumolib import checkBinary  # Checks for the binary in environ vars
import traci


def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


# contains TraCI control loop
def run():

    # Setting durations
    default_dur = 60
    max_dur = 120
    min_dur = 30
    yellow_dur = 30
    starve_thresh = 180
    
    tlsIDs = traci.trafficlight.getIDList()

    """
    Note on tl logic types:
    type:	enum (static, actuated, delay_based);	The type of the traffic light (fixed phase durations, 
    phase prolongation based on time gaps between vehicles (actuated), or on accumulated time loss of queued vehicles (delay_based) )
    """
    
    priorities = np.zeros(4, int)
    nextYellow = False                      # Used to transition to yellow phase

    starve_count = np.zeros(4, float)
    deltaT = traci.simulation.getDeltaT()    # Gets length of a simulation step
    timer = 0

    detectors = traci.lanearea.getIDList()
    d0 = detectors[0]
    step_count = 0
    print("Step\tJamLength\tMeanSpead\tOccupancy")
    while traci.simulation.getMinExpectedNumber() > 0:

        print(step_count, traci.lanearea.getJamLengthVehicle(d0), 
            traci.lanearea.getLastStepMeanSpeed(d0), 
            traci.lanearea.getLastStepOccupancy(d0), 
            sep='\t')

        if(timer - deltaT < 0):

            if(nextYellow) :

                curr_phase = traci.trafficlight.getPhase(tlsIDs[0])
                assert curr_phase % 2 == 0, "Already in a yellow phase"
                traci.trafficlight.setPhase(tlsIDs[0], curr_phase + 1)
                traci.trafficlight.setPhaseDuration(tlsIDs[0], yellow_dur)
                timer = yellow_dur
                nextYellow = False

            else:
                
                # Assign priority to each lane
                priorities =  np.ndarray.astype(starve_count - np.min(starve_count), int)    # For now most starved lanes have highest priority

                # Account for starved lanes
                starved_lanes = np.argwhere(starve_count > starve_thresh)

                if not len(starved_lanes):

                    # Assign green to highest priority lane
                    lane_idx = np.argmax(priorities)

                else:

                    # Assign green to starved lane with highest priority
                    starved_priorities = priorities[starved_lanes]
                    lane_idx = starved_lanes[np.argmax(starved_priorities)]

                traci.trafficlight.setPhase(tlsIDs[0], lane_idx*2)
                traci.trafficlight.setPhaseDuration(tlsIDs[0], default_dur)
                starve_count[lane_idx] = 0
                timer = default_dur
                nextYellow = True

        traci.simulationStep()
        step_count += 1
        timer -= deltaT
        starve_count +=  deltaT
        curr_lane = traci.trafficlight.getPhase(tlsIDs[0]) // 2
        starve_count[curr_lane] -= deltaT

    traci.close()
    sys.stdout.flush()


# main entry point
if __name__ == "__main__":
    options = get_options()
    
    # check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # traci starts sumo as a subprocess and then this script connects and runs
    traci.start([sumoBinary, "-c", "single_intersection_tls.sumo.cfg",
                             "--tripinfo-output", "single_intersection_tls.trips.xml"])
    run()

    # Calculating trip statistics
    parser = make_parser()

    allvehicles = {}
    vehfile = "single_intersection_tls.trips.xml"
    for filename in vehfile.split(","):
        allvehicles[filename] = []
        parser.setContentHandler(statisticsElements.VehInformationReader(allvehicles[filename]))
        parser.parse(filename)

    assignments = {}
    # calculate/read the basic statistics
    for method, vehicles in allvehicles.items():
        utils.getBasicStats(False, method, vehicles, assignments)

    print(
        'average vehicular travel time(s) = the sum of all vehicular travel times / the number of vehicles\n')
    print(
        'average vehicular travel length(m) = the sum of all vehicular travel lengths / the number of vehicles\n')
    print(
        'average vehicular travel speed(m/s) = the sum of all vehicular travel speeds / the number of vehicles\n')
    for method in assignments.values():
        print('\nAssignment Method:%s\n' % method.label)
        print('- total number of vehicles:%s\n' % method.totalVeh)
        print('- total departure delay(s):%s, ' %
                      method.totalDepartDelay)
        print('- average departure delay(s):%s\n' %
                      method.avgDepartDelay)
        print('- total waiting time(s):%s, ' % method.totalWaitTime)
        print('- average vehicular waiting time(s):%s\n' %
                      method.avgWaitTime)
        print('- total travel time(s):%s, ' % method.totalTravelTime)
        print('- average vehicular travel time(s):%s\n' %
                      method.avgTravelTime)
        print('- total travel length(m):%s, ' %
                      method.totalTravelLength)
        print('- average vehicular travel length(m):%s\n' %
                      method.avgTravelLength)
        print('- average vehicular travel speed(m/s):%s\n' %
                      method.avgTravelSpeed)