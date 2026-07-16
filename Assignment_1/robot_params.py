pioneer_wheel_radius = 0.0975 #[m] Radius of the wheels on each pioneer
pioneer_track_width = 0.4  #[m] Distance betweent he two wheels of pioneer

pioneer_max_W = 0.3 #[rad/seconds] Maximum angular velocity (omega) of pioneer
pioneer_max_V = 0.7 #[m/seconds] Maximum linear velocity (V) of pioneer

goal_threshold = 0.3 #[m] The threshold distance at whihc robot is declared to be at goal

#Names of the 16 ultrasonic sensors on Pioneer1's sonar ring (front 8 + rear 8),
#in scene-object order; index i here corresponds to sensor i+1
#Confirmed against the real scene 2026-07-16 - the stock Pioneer_p3dx prefix
#was kept (not renamed to Pioneer1_*)
pioneer_sensor_names = ["Pioneer_p3dx_ultrasonicSensor%d" % i for i in range(1, 17)]