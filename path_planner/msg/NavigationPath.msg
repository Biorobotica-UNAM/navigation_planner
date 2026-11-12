# Internal motion command; legacy nav_msgs/Path clients remain supported.
nav_msgs/Path path
uint8 FOLLOW_PATH=0
uint8 ROTATE=1
uint8 BRAKE=2
uint8 RECOVER_START=3
uint8 mode
# Optional arrival circle in the path frame; zero radius uses normal arrival.
geometry_msgs/Point arrival_center
float64 arrival_radius
