# Internal motion command; legacy nav_msgs/Path clients remain supported.
nav_msgs/Path path
# Optional arrival circle in the path frame; zero radius uses normal arrival.
geometry_msgs/Point arrival_center
float64 arrival_radius
