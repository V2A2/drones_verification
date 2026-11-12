process:
  cmd: >
    . ../ros2_ws/install/setup.sh &&
    python3 drone_as2_random_walk.py
    {% if ns | default('') %}-n {{ ns }}{% endif %}
    {% if seed is defined %}--seed {{ seed }}{% endif %}
  interactive: True
