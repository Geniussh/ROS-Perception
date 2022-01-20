# Launch the Colour Range detector
rosrun blob_tracking_v2 range_detector.py --filter HSV --preview
# Set tosee onlythe red haro:
# H_MIN = 0
# S_MIN = 234
# V_MIN = 0
# H_MAX = 0
# S_MAX = 255
# V_MAX = 255


# Move haro
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/haro/cmd_vel


rosrun blob_tracking_v2 blob_detector.py
rostopic echo /blob/point_blob
