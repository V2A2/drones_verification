# Demo: four drones in the baylands world, each with its state estimate and reachability verifier, flying a random
# walk. The random walk missions start together with the drones; each one waits by itself until its PX4 instance is up
# and ready to arm (px4_drone random_walk), so no warm-up delay is needed.
#
#   python3 simd/simd.py sim demo.mod -c verifier=fl
#
# Context (optional):
#   verifier  reachability checker, ps (ProbStar) or fl (face lifting) (default: ps)

{%- set drones = [
  ('drone1', '0,0,1'),
  ('drone2', '-5,10,1'),
  ('drone3', '-10,20,1'),
  ('drone4', '-15,30,1'),
] %}

requires:
{%- for ns, pose in drones %}
  - drone:
      ns: {{ ns }}
      instance: {{ loop.index0 }}
      world: baylands
      pose: '{{ pose }}'
      verifier: {{ verifier | default('ps') }}
  - ros2_ws/random_walk:
      ns: {{ ns }}
{%- endfor %}
