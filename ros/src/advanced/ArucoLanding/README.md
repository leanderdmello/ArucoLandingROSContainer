# ArucoLanding
src/aruco_pose_estimator.*: marker detection & pose estimation

src/landing_planner.*: generates full landing path with waypoints

src/landing_controller.*: decides next waypoint

src/main.cpp: node setup 

The system is composed of three main components:

ArucoPoseEstimator
Detects ArUco markers from the onboard camera feed using OpenCV. It uses multiple marker IDs (24 for the inner marker and 122 for the outer one), prioritising the smaller, central marker for better accuracy at close range. Pose estimates are calculated relative to the marker’s centre and averaged over a 3 seconds to reduce noise and improve stability.

LandingPlanner
Uses the current pose to generate a sequence of waypoints that guide the drone to land accurately in the centre of the platform. It creates a stepped descent path, lowering the drone’s altitude (z) while re-centring the drone to x = 0 and y = 0 at each step if necessary. The descent continues until the drone reaches the predefined target height (5 cm above the ground (this will be changed depending on the exact mounting of the camera)).

LandingController
Manages the descent stages and enforces speed limits and positional tolerances. It ensures that the drone only descends further when it is properly aligned above the marker, preventing drifting during landing.
