# Changelog

## Unreleased

- Add local RH56DFX velocity and force limiting to the G1 service.
- Publish measured actuator force through `MotorState.tau_est`.
- Block additional closure when feedback is unavailable and back off after
  contact-force overshoot.
- Add unloaded force calibration and state-only monitoring CLI modes.

## 1.0.1

+ Initial release.

## 1.2

- support unitree g1
