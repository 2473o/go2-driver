ros2 run go2_driver mcf_reset --ros-args \
  -p network_interface:=enp2s0 \
  -p service_name:=mcf \
  -p target_mode:=classic \
  -p restart_delay_ms:=2000 \
  -p timeout_sec:=10.0
