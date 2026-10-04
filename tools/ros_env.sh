# ROS environment for the sim container (source it inside `docker exec <c> bash -lc "...; ..."`).
# A plain exec shell has no DDS settings and sees an EMPTY graph; these match the user's terminals.
source ~/colcon_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///home/rg-station-04-keith/colcon_ws/src/sobit_home/cyclonedds_local.xml ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
