#!/bin/sh

# http://www.ros.org/wiki/rostopic#rostopic_pub
X=${1:-100}
Y=${2:-200}
rostopic pub -1 /image_painted/screenpoint geometry_msgs/PointStamped "{point: {x: $X, y: $Y}}"

