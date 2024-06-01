puzzlebot@jetson:~/proyecto_final$ ros2 run proyecto_final velocidad 
Traceback (most recent call last):
  File "/home/puzzlebot/proyecto_final/install/proyecto_final/lib/proyecto_final/velocidad", line 11, in <module>
    load_entry_point('proyecto-final==0.0.0', 'console_scripts', 'velocidad')()
  File "/home/puzzlebot/proyecto_final/install/proyecto_final/lib/python3.8/site-packages/proyecto_final/velocidad.py", line 47, in main
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
  File "/home/puzzlebot/proyecto_final/install/proyecto_final/lib/python3.8/site-packages/proyecto_final/velocidad.py", line 40, in timer_callback
    self.robot_vel.angular.z = 0
  File "/opt/ros/humble/lib/python3.8/site-packages/geometry_msgs/msg/_vector3.py", line 165, in z
    assert \
AssertionError: The 'z' field must be of type 'float'
[ros2run]: Process exited with failure 1
