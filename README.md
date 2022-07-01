# ROS_HELPER_FUNCTIONS

tf_broadcaster - combines tf lookup with a position topic to republish to a global position topic

static_tf_publisher.launch - for quick reference of the built in tf lookup static transformer node

slow_yaw - module to enable a mavros uav to yaw at a controlled rate to prevent visual localisation drift

position_benchmark - benchmark 2 positional sources to check accuracy (ie measured vs ground truth)

## Borealis specific

mock_multiplexer(depreciated) - used to control multiple drone input sources into one output

mode_manager - mux for a posestamped topic(more generically), able to be used directly for offboard control to mavros
