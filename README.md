# smart-traffic-management
Optimizing traffic lights to increase traffic throughput in urban areas

## Useful Commands

* To create grid network xml file:
    netgenerate --grid --grid.x-number=3 --grid.y-number=3 --grid.x-length=150 --grid.y-length=150 -o single_intersection_tls.net.xml

* Command to generate random traffic in road network:
    python3 $SUMO_HOME/tools/randomTrips.py -n "single_intersection_tls.net.xml" -o "single_intersection_tls.trips.xml" -r "single_intersection_tls.routes.xml" -e 1000 --fringe-factor 10

## SUMO information

* Phase state format:
..2. G : Green (go)
..3. y : Yello (go slow)
..4. r : Red (stop)
..5. g : Green (go, minor conflict)

* South -> East -> North -> West

* Order of colors for each direction:

... Left turn -> Through, Right turn -> U-turn (For left-hand traffic)



