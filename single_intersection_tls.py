#!/usr/bin/env python

import os
import sys
import optparse
import numpy as np
import utils
import pandas as pd

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
def run(priority_mode = 0):

    # Initializations
    default_dur = 60
    max_dur = 120
    min_dur = 30
    yellow_dur = 30
    starve_thresh = 180
    priorities = np.zeros(4, int)
    nextYellow = False                      # Used to transition to yellow phase
    starve_count = np.zeros(4, float)
    deltaT = traci.simulation.getDeltaT()    # Gets length of a simulation step
    timer = 0
    step_count = 0    
    tlsIDs = traci.trafficlight.getIDList()
    controlled_lanes = traci.trafficlight.getControlledLanes(tlsIDs[0])

    while traci.simulation.getMinExpectedNumber() > 0:

        if(timer - deltaT < 0):

            if(nextYellow) :

                curr_phase = traci.trafficlight.getPhase(tlsIDs[0])
                assert curr_phase % 2 == 0, "Already in a yellow phase"
                traci.trafficlight.setPhase(tlsIDs[0], curr_phase + 1)
                traci.trafficlight.setPhaseDuration(tlsIDs[0], yellow_dur)
                timer = yellow_dur
                nextYellow = False

            else:
                
                # Getting lane stats
                jam_lengths = np.array(list(map(
                    lambda x : traci.lanearea.getJamLengthVehicle('e2det_' + controlled_lanes[x*4]), 
                    range(4)
                )))

                lane_occupance = np.array(list(map(
                    lambda x : traci.lanearea.getLastStepOccupancy('e2det_' + controlled_lanes[x*4]), 
                    range(4)
                )))

                lane_speed = np.array(list(map(
                    lambda x : traci.lanearea.getLastStepMeanSpeed('e2det_' + controlled_lanes[x*4]), 
                    range(4)
                )))

                # Assign priority to each lane
                if priority_mode == 0:
                    priorities =  np.ndarray.astype(starve_count - np.min(starve_count), int)    # For now most starved lanes have highest priority
                elif priority_mode == 1:
                    priorities = jam_lengths
                elif priority_mode == 2:
                    priorities = lane_occupance
                elif priority_mode == 3:
                    priorities = lane_speed
                elif priority_mode == 4:
                    priorities = lane_speed * lane_occupance
                elif priority_mode == 5:
                    priorities = lane_speed * jam_lengths

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

    df_columns=['VehicleCount', 'TotalDepartureDelay', 'TotalWaitingTime', 'TotalTravelTime', 'TotalTravelLength',
    'AverageDepartureDelay', 'AverageWaitingTime', 'AverageTravelTime', 'AverageTravelLength', 'AverageTravelSpeed']
    df = pd.DataFrame(columns=df_columns)

    for priority_mode in range(6):

        # traci starts sumo as a subprocess and then this script connects and runs
        traci.start([sumoBinary, "-c", "single_intersection_tls.sumo.cfg",
                                "--tripinfo-output", "single_intersection_tls.trips.xml"])
        run(priority_mode)

        # Calculating trip statistics
        stats = utils.getBasicStats("single_intersection_tls.trips.xml")
        df = pd.concat([
            df, pd.DataFrame(data=[[
                stats.totalVeh, stats.totalDepartDelay, 
                stats.totalWaitTime, stats.totalTravelTime, stats.totalTravelLength, stats.avgDepartDelay, 
                stats.avgWaitTime, stats.avgTravelTime, stats.avgTravelLength, stats.avgTravelSpeed
            ]], columns=df_columns)
        ], ignore_index=True)

    print(df[['TotalWaitingTime', 'TotalTravelTime', 'AverageTravelSpeed']])