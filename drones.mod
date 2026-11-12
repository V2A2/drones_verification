requires:
  - drone:
      ns: drone1
      instance: 0
      world: baylands
      pose: '0,0,1'
      verifier: {{ verifier | default('ps') }}
  - drone:
      ns: drone2
      instance: 1
      world: baylands
      pose: '-5,10,1'
      verifier: {{ verifier | default('ps') }}
  - drone:
      ns: drone3
      instance: 2
      world: baylands
      pose: '-10,20,1'
      verifier: {{ verifier | default('ps') }}
  - drone:
      ns: drone4
      instance: 3
      world: baylands
      pose: '-15,30,1'
      verifier: {{ verifier | default('ps') }}
