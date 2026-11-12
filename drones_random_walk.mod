requires:
  - mission_scripts/drone_as2_random_walk:
      ns: "drone1"
      seed: {{ (seed | default(0)) * 10 + 1 }}
  - mission_scripts/drone_as2_random_walk:
      ns: "drone2"
      seed: {{ (seed | default(0)) * 10 + 2 }}
  - mission_scripts/drone_as2_random_walk:
      ns: "drone3"
      seed: {{ (seed | default(0)) * 10 + 3 }}
  - mission_scripts/drone_as2_random_walk:
      ns: "drone4"
      seed: {{ (seed | default(0)) * 10 + 4 }}
