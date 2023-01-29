# smart-traffic-management
Optimizing traffic lights to increase traffic throughput in urban areas

## Useful Commands

* To create grid network xml file:
    netgenerate --grid --grid.x-number=3 --grid.y-number=3 --grid.x-length=150 --grid.y-length=150 -o single_intersection_tls.net.xml

* Command to generate random traffic in road network:
    python3 $SUMO_HOME/tools/randomTrips.py -n "single_intersection_tls.net.xml" -o "single_intersection_tls.trips.xml" -r "single_intersection_tls.routes.xml" -e 1000 --fringe-factor 10

* Command to generate e2 detectors at traffic lights:
    python3 $SUMO_HOME/tools/output/generateTLSE2Detectors.py -n single_intersection_tls.net.xml -o e2.add.xml

## SUMO information

* Phase state format:
    rugGyYuoO, for red, red-yellow, green, yellow, off, where lower case letters mean that the stream has
    to decelerate.

* South -> East -> North -> West

* Order of colors for each direction:

... Left turn -> Through, Right turn -> U-turn (For left-hand traffic)

## Description of Statistics

* average vehicular travel time(s) = the sum of all vehicular travel times / the number of vehicles
* average vehicular travel length(m) = the sum of all vehicular travel lengths / the number of vehicles
* average vehicular travel speed(m/s) = the sum of all vehicular travel speeds / the number of vehicles



