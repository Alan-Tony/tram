#!/usr/bin/env python

import os
import sys
import optparse
import numpy as np

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

    # Preparing phase for logic0
    phases = [
        traci.trafficlight.Phase(duration=default_dur, state= "GGGGrrrrrrrrrrrr", minDur=min_dur, maxDur=max_dur),
        traci.trafficlight.Phase(duration=yellow_dur, state= "yyyyrrrrrrrrrrrr", minDur=yellow_dur, maxDur=yellow_dur),
        traci.trafficlight.Phase(duration=default_dur, state= "rrrrGGGGrrrrrrrr", minDur=min_dur, maxDur=max_dur),
        traci.trafficlight.Phase(duration=yellow_dur, state= "rrrryyyyrrrrrrrr", minDur=yellow_dur, maxDur=yellow_dur),
        traci.trafficlight.Phase(duration=default_dur, state= "rrrrrrrrGGGGrrrr", minDur=min_dur, maxDur=max_dur),
        traci.trafficlight.Phase(duration=yellow_dur, state= "rrrrrrrryyyyrrrr", minDur=yellow_dur, maxDur=yellow_dur),
        traci.trafficlight.Phase(duration=default_dur, state= "rrrrrrrrrrrrGGGG", minDur=min_dur, maxDur=max_dur),
        traci.trafficlight.Phase(duration=yellow_dur, state= "rrrrrrrrrrrryyyy", minDur=yellow_dur, maxDur=yellow_dur)
    ]

    # Create new program logic for the traffic light
    traci.trafficlight.setProgramLogic(tlsIDs[0], traci.trafficlight.Logic(
        programID="0", type=0, currentPhaseIndex=0, phases=phases
    ))
    
    priorities = np.zeros(4, int)
    nextYellow = False                      # Used to transition to yellow phase

    starve_count = np.zeros(4, float)
    deltaT = traci.simulation.getDeltaT()    # Gets length of a simulation step
    timer = 0
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