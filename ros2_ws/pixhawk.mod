requires:
  - {{ search_module("micro-xrce-dds-agent") }}

process:
  cmd: >
    . install/setup.sh && 
    ros2 launch as2_bringup px4_ros2_bridge.launch.py
    {% if ns | default('') %}namespace:={{ ns }}{% endif %}
    fmu_prefix:={{ '/' + ( ns | default('')) }}
    target_system_id:={{ ( instance | default(0) ) + 1 }}