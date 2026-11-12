process:
  cmd: >
    . install/setup.sh &&
    ros2 launch fl_reach fl_reach.launch.py
    {% if ns | default('') %}namespace:={{ ns }}{% endif %}
    system_id:={{ (instance | default(0)) + 1 }}
  interactive: True
