# Changelog

## Unreleased

- Add local RH56DFX velocity and force limiting to the G1 service.
- Publish measured actuator force through `MotorState.tau_est`.
- Block additional closure when feedback is unavailable and back off after
  contact-force overshoot.
- Add unloaded force calibration and state-only monitoring CLI modes.
- Add verified low-speed opening, persistent median force baselines, filtered
  contact confirmation, and open-state-only drift compensation.
- Report force-calibration acknowledgements when available while supporting
  firmware that returns none, and preserve serial read timeouts across
  repeated transactions.

## 1.0.1

+ Initial release.

## 1.2

- support unitree g1
