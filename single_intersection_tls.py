#!/usr/bin/env python

import os
import sys
import optparse

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

    #Setting maximum and minimum phase durations
    max_dur= 120
    min_dur= 30
    
    tlsID = traci.trafficlight.getIDList()

    # Preparing phase for logic0
    phases = [
        traci.trafficlight.Phase(duration=min_dur, state= "GGGGrrrrrrrrrrrr", minDur=min_dur, maxDur=max_dur),
        traci.trafficlight.Phase(duration=min_dur, state= "yyyyrrrrrrrrrrrr", minDur=min_dur, maxDur=max_dur),
        traci.trafficlight.Phase(duration=min_dur, state= "rrrrGGGGrrrrrrrr", minDur=min_dur, maxDur=max_dur),
        traci.trafficlight.Phase(duration=min_dur, state= "rrrryyyyrrrrrrrr", minDur=min_dur, maxDur=max_dur),
        traci.trafficlight.Phase(duration=min_dur, state= "rrrrrrrrGGGGrrrr", minDur=min_dur, maxDur=max_dur),
        traci.trafficlight.Phase(duration=min_dur, state= "rrrrrrrryyyyrrrr", minDur=min_dur, maxDur=max_dur),
        traci.trafficlight.Phase(duration=min_dur, state= "rrrrrrrrrrrrGGGG", minDur=min_dur, maxDur=max_dur),
        traci.trafficlight.Phase(duration=min_dur, state= "rrrrrrrrrrrryyyy", minDur=min_dur, maxDur=max_dur)
    ]

    """
    Note on tl logic types:
    type:	enum (static, actuated, delay_based);	The type of the traffic light (fixed phase durations, 
    phase prolongation based on time gaps between vehicles (actuated), or on accumulated time loss of queued vehicles (delay_based) )
    """
    # Create new program logic for the traffic light
    traci.trafficlight.setProgramLogic(tlsID[0], traci.trafficlight.Logic(
        programID="0", type=0, currentPhaseIndex=0, phases=phases
    ))
    
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:

        traci.simulationStep()
        step += 1

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