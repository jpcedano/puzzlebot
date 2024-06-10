Traceback (most recent call last):
  File "/home/puzzlebot/proyecto_final/install/proyecto_final/lib/proyecto_final/odometry", line 11, in <module>
    load_entry_point('proyecto-final==0.0.0', 'console_scripts', 'odometry')()
  File "/home/puzzlebot/proyecto_final/install/proyecto_final/lib/python3.8/site-packages/proyecto_final/odometry.py", line 78, in main
    rclpy.spin(odometry_subscriber)
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
  File "/opt/ros/humble/lib/python3.8/site-packages/rclpy/executors.py", line 362, in _execute_subscription
    await await_or_execute(sub.callback, msg)
  File "/opt/ros/humble/lib/python3.8/site-packages/rclpy/executors.py", line 107, in await_or_execute
    return callback(*args)
  File "/home/puzzlebot/proyecto_final/install/proyecto_final/lib/python3.8/site-packages/proyecto_final/odometry.py", line 45, in objetos_callback
    self.objeto_detectado = self.objeto_detectado[0]
IndexError: string index out of range
[ros2run]: Process exited with failure 1
