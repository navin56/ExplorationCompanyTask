The Exploration Company Data Processing Task.

Task Overview:
1.  Three Input Sensors
    - IMU
    - GNSS
    - Star Tracker

2. Outputs
    - 6x Thrusters

3. GNC Stub Function
    - Takes the input and outputs on/off State for the thrusters.
    - Input data can be null.
    - Multiple rates implied for sensors here.

Goals:
1. Scable, Reusable Software Framework
    - Adding Functionality with less interface level changes.
    - Reuse with diffrent hardwares.
2. 

Assumptions:

1. Sensors
    - Generally the IMU has the higher sampling rates. 
    - Assume the IMU sampling is done at 100 Hz. 0.01s
    - All the sensors are interfaced to unique hardware serial interfaces.
    - The GNSS generally has a 1Hz sampling rate, although higher rates are often possible.
        - Assume the GNSS sampling is 10Hz. 0.1s.
    - Star Trackers have low sampling frequencies.
        - Assume the Star tracker rate is 1Hz. 1s.

    - The time-synchronization logic between these sensors is *not considered*.
        - This is required by navigation. However it is very device specific.

2.