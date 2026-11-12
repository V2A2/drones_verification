process:
  cmd: >
    . install/setup.sh &&
    ros2 launch probstar_reach probstar_reach.launch.py 
    system_id:={{ (instance | default(0)) + 1 }}
    {% if ns | default('') %}namespace:={{ ns }}{% endif %}
  interactive: True
