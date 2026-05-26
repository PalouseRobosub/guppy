# guppy_localization

This package contains the sensor fusion code, as well as the CAN acquisition code to read the barometer data. Additionally, these launch files also start up the DVL and IMU.

Several of Guppy’s sensors are COTS sensors we have either bought or inherited from previous teams. For an IMU, we have a sponsored VN-100 from VectorNav Technologies which provides us with high-resolution inertial measurements and orientation data. We also use a WaterLinked DVL-A50 as our Doppler Velocity Log, which provides us with very accurate relative velocity data, as well as dead-reckoning position estimates. Both these sensors communicate directly with our single-board-computer, the LattePanda Sigma from DFRobot.

Additional sensors include two rotary switches from Blue Robotics and bolt-style reed switches from McMaster Carr which serve as generic GPIO inputs and power switches. We also incorporate a Blue Robotics  High-Resolution Depth/Pressure sensor for accurate Z-axis measurements. We also rely on several Logitech USB webcams, as well as several USB and Ethernet Flir/PointGrey high resolution cameras for our vision system.
