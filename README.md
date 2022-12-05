# smart-traffic-management
Optimizing traffic lights to increase traffic throughput in urban areas

## Useful Commands

* To create grid network xml file:
    netgenerate --grid --grid.x-number=3 --grid.y-number=3 --grid.x-length=150 --grid.y-length=150 -o single_intersection_tls.net.xml

* Command to generate random traffic in road network:
    python3 $SUMO_HOME/tools/randomTrips.py -n "single_intersection_tls.net.xml" -o "single_intersection_tls.trips.xml" -r "single_intersection_tls.routes.xml" -e 1000 --fringe-factor 10

