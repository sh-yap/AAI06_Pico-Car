# Buddy 2 evidence

Store dated CSV exports here. Use this header so motor, encoder, PID, distance,
turn and fault runs can be compared after final-car assembly:

```text
timestamp_us,test_id,sequence_id,target_left_mm_s,target_right_mm_s,measured_left_mm_s,measured_right_mm_s,left_counts,right_counts,left_pwm_permille,right_pwm_permille,motion_state,fault,battery_voltage_v
```

No file in this directory should be interpreted as final-car verification until
the test setup, calibration constants and vehicle configuration are recorded.
