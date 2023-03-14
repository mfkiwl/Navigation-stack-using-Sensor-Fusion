# Navigation-stack-using-Sensor-Fusion

Buiding a Navigation stack using sensors - GNSS module and VectorNav VN-100 IMU to understand their relative strengths and drawbacks.

This project contains the creation of a navigation stack using sensor fusion involved building device drivers in Python for data acquisition from a GNSS module and a VectorNav VN-100 IMU. These drivers were integrated into a single launch file in a ROS environment and used to collect data from the sensors. The collected data was then analyzed using MATLAB, including magnetometer calibration and dead reckoning calculations.

# Analysis of the data collected in a car for dead-reckoning

# 1. Data collected by Car donuts

The data is collected by driving the car in circles for calibrating the IMU sensor.

Calibrating the Inertial Measurement Unit (IMU) sensor is important for accurate navigation and control of autonomous vehicles. The IMU sensor measures the vehicle's acceleration, angular velocity, and orientation, which are crucial for determining the vehicle's position and heading.

When calibrating the IMU sensor, the vehicle needs to be placed in different positions and orientations to ensure that the sensor's readings are accurate in all directions. One common method for calibrating the IMU sensor is to drive the car in circles.

By driving the car in circles, the sensor is exposed to different rates of rotation and acceleration in different directions, allowing it to accurately calibrate its readings. The circles also allow for the calibration of the sensor's biases, which are errors that are present in the sensor's readings due to manufacturing tolerances, environmental factors, or other sources.

Overall, driving the car in circles is a simple and effective way to calibrate the IMU sensor, ensuring that it provides accurate readings for the vehicle's navigation and control systems.

Procedure:
Begin a rosbag and call it data_going_in_circles.bag
- Wait 10 - 15 seconds
- Start your car
- Drive the car in circles 4-5 times (the more circular the path, the better). One
suggested place is the Ruggles circle near Centennial common.
- Stop recording only the rosbag. Do not turn off the car or the driver nodes.

2. Mini Route

Procedure:
Begin a rosbag and call it data_driving.bag.
- You might want to make a video of your route as you drive, it may help
understand any data anomalies.
- Wait 10 - 15 seconds.
- Go for a drive around Boston AND return to the spot where you started. We
suggest a total distance of ATLEAST 2 – 3 kilometers with a MINIMUM of 10
turns.
- Since this is a tour, do not go underground/in tunnels. Enjoy the view (whether
outdoors or on the linux screen).
- Once you return to the spot you started from, stop the rosbag recording
- You can now turn off your car & your ROS drivers.

# 1. Estimate the heading (yaw)
Magnetometer Calibration:
 Correct magnetometer readings for "hard-iron" and "soft-iron" effects using the data collected
when going around in circles. Do not use magcal (a built-in function of MATLAB). Write the code
yourself. The magcal function is not optimized for high-grade sensors.
 Submit a plot showing the magnetometer data before and after the correction in your report.

