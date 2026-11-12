requires:
  - px4_gz:
      instance: {{ instance | default(0) }}
      ns: "{{ ns | default('') }}"
      model: {{ model | default('x500') }}
      pose: {{ pose | default('0,0,1') }}
      world: {{ world | default('empty') }}
  - ros2_ws/pixhawk:
      instance: {{ instance | default(0) }}
      ns: "{{ ns | default('') }}"
  - ros2_ws/drone_as2:
      ns: "{{ ns | default('') }}"
  - ros2_ws/verifier_{{ verifier | default('ps') }}:
      instance: {{ instance | default(0) }}
      ns: "{{ ns | default('') }}"