# PX4 EKF2 state of one drone in the shared earth frame (px4_drone state_estimate): <ns>/state/pose, <ns>/state/twist.
# Runs for the whole session; the verifiers and the viewer read it.
requires:
  - {{ search_module("micro-xrce-dds-agent") }}

process:
  cmd: >
    . install/setup.sh &&
    ros2 launch px4_drone state_estimate.launch.py
    {% if ns | default('') %}namespace:={{ ns }}{% endif %}
  interactive: True
