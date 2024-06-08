[INFO] [1717829921.333815611] [TrafficLightDetector_node]: Traffic Light Detector Node started
Traceback (most recent call last):
  File "/home/puzzlebot/proyecto_final/install/proyecto_final/lib/proyecto_final/semaforos", line 11, in <module>
    load_entry_point('proyecto-final==0.0.0', 'console_scripts', 'semaforos')()
  File "/home/puzzlebot/proyecto_final/install/proyecto_final/lib/python3.8/site-packages/proyecto_final/semaforos.py", line 108, in main
    rclpy.spin(node)
  File "/opt/ros/humble/lib/python3.8/site-packages/rclpy/__init__.py", line 222, in spin
    executor.spin_once()
  File "/opt/ros/humble/lib/python3.8/site-packages/rclpy/executors.py", line 739, in spin_once
    self._spin_once_impl(timeout_sec)
  File "/opt/ros/humble/lib/python3.8/site-packages/rclpy/executors.py", line 736, in _spin_once_impl
    raise handler.exception()
  File "/opt/ros/humble/lib/python3.8/site-packages/rclpy/task.py", line 239, in __call__
    self._handler.send(None)
  File "/opt/ros/humble/lib/python3.8/site-packages/rclpy/executors.py", line 437, in handler
    await call_coroutine(entity, arg)
  File "/opt/ros/humble/lib/python3.8/site-packages/rclpy/executors.py", line 351, in _execute_timer
    await await_or_execute(tmr.callback)
  File "/opt/ros/humble/lib/python3.8/site-packages/rclpy/executors.py", line 107, in await_or_execute
    return callback(*args)
  File "/home/puzzlebot/proyecto_final/install/proyecto_final/lib/python3.8/site-packages/proyecto_final/semaforos.py", line 41, in timer_callback
    result_frame, signal_value = self.detect_colored_circles(frame)
  File "/home/puzzlebot/proyecto_final/install/proyecto_final/lib/python3.8/site-packages/proyecto_final/semaforos.py", line 103, in detect_colored_circles
    return frame, signal_value
UnboundLocalError: local variable 'signal_value' referenced before assignment
[ros2run]: Process exited with failure 1
