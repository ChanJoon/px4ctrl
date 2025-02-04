#!/bin/bash
sleep 3
# Run the first rosrun command and wait 1 second
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
# Run the second rosrun command and wait 1 second
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
