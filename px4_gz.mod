# PX4 SITL vehicle template. Depend on it as `px4_gz` with a context — module
# identity comes from a hash of the rendered content, so each distinct
# context automatically becomes its own instance, no manual id needed. The
# numeric PX4 instance is passed explicitly as `instance` — it feeds
# `px4 -i`, which offsets the MAVLink/uXRCE ports, so it must be unique per
# vehicle. Example:
#
#   wants:
#     - px4_gz:
#         instance: 0
#     - px4_gz:
#         instance: 1
#         model: x500
#         pose: "2,0,0"
#
# Context vars:
#   instance   numeric PX4 instance id for `px4 -i`   (default: 0; must be unique per vehicle)
#   ns         ROS 2 namespace (PX4_UXRCE_DDS_NS)     (default: unset, no namespace)
#   model      gz model name without the gz_ prefix   (default: x500)
#   pose       spawn pose x,y,z[,roll,pitch,yaw]      (default: 0,1,1)
#   world      gz world the vehicle spawns into       (default: default)
#   autostart  PX4_SYS_AUTOSTART airframe id          (default: 4001 = gz x500)
#
# Runs from PX4-Autopilot/, where the px4_sitl_default build lives.
# Same invocation as utils/sitl/run_instance.bash: the host-built px4 binary
# with PX4_GZ_STANDALONE=1, spawning into the already-running gz world (the
# gazebo dep below). GZ_PARTITION/GZ_IP must match the gazebo module or
# gz_bridge can't discover the world. -d disables the interactive pxh shell,
# which can't be shared when simd multiplexes several instances' output.
process:
  cmd: >
    cd PX4-Autopilot &&
    GZ_PARTITION=px4_sim GZ_IP=127.0.0.1
    PX4_GZ_STANDALONE=1
    PX4_UXRCE_DDS_NS={{ ns | default('') }}
    PX4_SIM_MODEL=gz_{{ model | default('x500') }}
    PX4_GZ_MODEL_POSE='{{ pose | default('0,0') }}'
    PX4_GZ_WORLD={{ world | default('empty') }}
    build/px4_sitl_default/bin/px4 -i {{ instance | default(0) }}
  interactive: True

# The world var also selects the gazebo module instance, so the world the
# vehicle spawns into and the world gz actually runs can never diverge.
requires:
  - gazebo:
      world: {{ world | default('default') }}
