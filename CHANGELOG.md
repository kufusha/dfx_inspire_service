# Changelog

## Unreleased

- Hold each actuator at its measured contact position during normal DDS
  control instead of reopening it, preventing sequential contacts from
  loosening a multi-finger grasp.

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
- Add an explicit force-limited return to the measured protective pose after
  calibration.
- Wait for post-calibration settling and use a rolling stable window for tare
  so isolated startup transients do not invalidate calibration.
- Preserve signed sensor offsets and pause protective-pose closure across
  isolated feedback timeouts, failing after repeated loss.
- Preserve per-transaction serial timeouts and identify the failed feedback
  transaction in protective-pose diagnostics.
- Allow 20 seconds for low-speed calibration opening and report every final
  actuator position when open verification fails.
- Permit standalone protective-pose return with a saved baseline and retry
  moving position feedback before issuing a hold.

## 1.0.1

+ Initial release.

## 1.2

- support unitree g1
