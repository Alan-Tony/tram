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
    
    tlsIDs = traci.trafficlight.getIDList()
    # Information of each traffic light
    timers = np.zeros(len(tlsIDs), float)
    # controlled_lanes = list(map(
    #     lambda x : tuple(np.unique(
    #         traci.trafficlight.getControlledLanes(x)
    #     )),
    #     tlsIDs
    # ))
    neighbor_map, controlled_lanes = utils.get_junction_info()
    priorities = np.zeros((len(tlsIDs), 6))
    nextYellow = [False] * len(tlsIDs)                      # Used to transition to yellow phase
    starve_count = np.zeros((len(tlsIDs), 6))
    curr_phase = [0] * len(tlsIDs)

    step_count = 0    
    deltaT = traci.simulation.getDeltaT()    # Gets length of a simulation step

    while traci.simulation.getMinExpectedNumber() > 0:

        for i, tlsID in enumerate(tlsIDs):

            lanes = controlled_lanes[tlsID]
        
            if(timers[i] - deltaT < 0):

                if(nextYellow[i]) :

                    curr_phase[i] = traci.trafficlight.getPhase(tlsID)
                    assert curr_phase[i] % 2 == 0, "Already in a yellow phase"
                    traci.trafficlight.setPhase(tlsID, curr_phase[i] + 1)
                    curr_phase[i] = curr_phase[i] + 1
                    traci.trafficlight.setPhaseDuration(tlsID, yellow_dur)
                    timers[i] = yellow_dur
                    nextYellow[i] = False

                else:
                    
                    # Getting lane stats
                    jam_lengths = np.array(list(map(
                        lambda x : traci.lanearea.getJamLengthVehicle('e2det_' + lanes[x]), 
                        range(len(lanes))
                    )))

                    lane_occupance = np.array(list(map(
                        lambda x : traci.lanearea.getLastStepOccupancy('e2det_' + lanes[x]), 
                        range(len(lanes))
                    )))

                    lane_speed = np.array(list(map(
                        lambda x : traci.lanearea.getLastStepMeanSpeed('e2det_' + lanes[x]), 
                        range(len(lanes))
                    )))

                    # Assign priority to each lane
                    if priority_mode == 0:
                        priorities[i] =  np.ndarray.astype(np.squeeze(starve_count[i]) - np.min(starve_count[i]), int)    # For now most starved lanes have highest priority
                    elif priority_mode == 1:
                        priorities[i] = jam_lengths
                    elif priority_mode == 2:
                        priorities[i] = lane_occupance
                    elif priority_mode == 3:
                        priorities[i] = lane_speed
                    elif priority_mode == 4:
                        priorities[i] = lane_speed * lane_occupance
                    elif priority_mode == 5:
                        priorities[i] = lane_speed * jam_lengths

                    # Account for starved lanes
                    starved_lanes = np.argwhere(np.squeeze(starve_count[i]) > starve_thresh)

                    if not len(starved_lanes):

                        # Assign green to highest priority lane
                        lane_idx = np.argmax(priorities[i])

                    else:

                        # Assign green to starved lane with highest priority
                        starved_priorities = priorities[i][starved_lanes]
                        lane_idx = starved_lanes[np.argmax(starved_priorities)]

                    traci.trafficlight.setPhase(tlsID, (lane_idx // 2) *2)    # The phase is set as the greatest even number less than lane_idx
                    curr_phase[i] = (lane_idx // 2) *2
                    traci.trafficlight.setPhaseDuration(tlsID, default_dur)
                    starve_count[i][(lane_idx // 2) * 2] = 0
                    starve_count[i][(lane_idx // 2) * 2 + 1] = 0
                    timers[i] = default_dur
                    nextYellow[i] = True
            
            starve_count[i] +=  deltaT
            starve_count[i][(curr_phase[i] // 2) * 2] -= deltaT
            starve_count[i][(curr_phase[i] // 2) * 2 + 1] -= deltaT

        traci.simulationStep()
        step_count += 1
        timers -= deltaT    # Subtract step time from all tls timers

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
        traci.start([sumoBinary, "-c", "exp.sumocfg",
                                "--tripinfo-output", "exp.trips.xml"])
        run(priority_mode)

        # Calculating trip statistics
        stats = utils.getBasicStats("exp.trips.xml")
        df = pd.concat([
            df, pd.DataFrame(data=[[
                stats.totalVeh, stats.totalDepartDelay, 
                stats.totalWaitTime, stats.totalTravelTime, stats.totalTravelLength, stats.avgDepartDelay, 
                stats.avgWaitTime, stats.avgTravelTime, stats.avgTravelLength, stats.avgTravelSpeed
            ]], columns=df_columns)
        ], ignore_index=True)

    df.insert(0, 'PriorityMode', ['Starvation Time', 'Queue Length', 'Lane Occupance', 'Lane Speed', 'Speed & Occupance', 'Speed & Length'])
    print(df[['PriorityMode', 'AverageWaitingTime', 'TotalWaitingTime', 'TotalTravelTime', 'AverageTravelSpeed']])