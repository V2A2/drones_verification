process:
  cmd: >
    . install/setup.sh &&
    ros2 launch as2_bringup drone_as2_bringup.launch.py
    {% if ns | default('') %}namespace:={{ ns }}{% endif %}
  interactive: True
