import robomaster as robot





ep_robot = robot.Robot()
ep_robot.initialize(conn_type="sta")
ep_chassis = ep_robot.chassis
ep_chassis.sub_position(freq=10, callback=robot_position)