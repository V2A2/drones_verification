# Random walk mission of one drone (px4_drone random_walk): a PX4 external mode that arms, takes off and flies random
# targets, aborting a target on <ns>/collision_detected. Started together with the drones (demo.mod): it retries the
# registration until PX4 is up and waits until the drone is ready to arm.
process:
  cmd: >
    . install/setup.sh &&
    ros2 launch px4_drone random_walk.launch.py
    {% if ns | default('') %}namespace:={{ ns }}{% endif %}
  interactive: True
