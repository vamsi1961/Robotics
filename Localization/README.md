# Filtering Localization

Simulation Setup: The code sets up parameters for the simulation, including noise values for control inputs (INPUT_NOISE), GPS measurements (GPS_NOISE), and the time step (DT).

System Dynamics and Motion Model: The system dynamics of a mobile robot are defined, including equations for position and orientation changes over time. The motion model (motion_model) is implemented to update the state of the robot based on control inputs.

Extended Kalman Filter (EKF) Functions:

compute_JA(x, u): Computes the Jacobian matrix jF used in the prediction step of the EKF.
get_covariance_ellipse(xEst, PEst): Calculates the covariance ellipse of the estimated state to visualize uncertainty.
prediction(xd, u, P): Implements the prediction step of the EKF, including state prediction and covariance update.
calc_input(): Generates control inputs (velocity and yaw rate) for the simulation.
Covariance Matrices:

Q: Represents the process noise covariance matrix.
R: Represents the measurement noise covariance matrix for GPS measurements.
Simulation - Dead Reckoning (DR): Simulates the motion of a mobile robot using the motion model and control inputs. Dead reckoning is used to estimate the robot's state over time. The trajectory and covariance ellipse of the estimate are visualized.

Simulation - Extended Kalman Filter (EKF): Incorporates GPS measurements into the estimation process using the EKF. The code updates the robot's estimated state and covariance based on GPS measurements and compares it to the ground truth trajectory.

Visualization: The code visualizes the Dead Reckoning (DR), Extended Kalman Filter (EKF), ground truth trajectory, and GPS measurements using animations. It shows how the estimates evolve over time and how they compare to the actual trajectory.

This code demonstrates the use of an Extended Kalman Filter for state estimation in the context of mobile robotics, showcasing the integration of noisy sensor measurements to improve state estimates.



# Particle Localization

Mobile Robot and Environment Setup:

The code simulates the behavior of a planar outdoor robot equipped with an IMU and an RF transponder.
The robot's dynamics are modeled as a unicycle.
The simulation runs at a frequency of 10 Hz for motion and measurement updates, and the RF transponder updates at 5 Hz.
The robot's state vector includes its position (x, y), orientation theta, and velocity v.
Uncertainty Parameters:

Q represents the uncertainty in the motion model.
R represents the uncertainty in range measurements.
Q_sim and R_sim are used to introduce simulation noise.
System Dynamics:

The robot's motion is defined using a set of kinematic equations.
The motion model predicts the robot's next state given control inputs.
Measurement Model:

The RF transponder on the robot measures distances to RFID tags.
The code calculates the expected distance to each tag based on the robot's current position and compares it to the measured distance.
Weight for each particle is calculated based on the Gaussian likelihood of the difference between measured and expected distances.
Resampling:

Low variance resampling is used to resample particles based on their weights.
Particles with larger weights are duplicated more.
Control Input and Prediction:

Control inputs are generated for the simulation.
Prediction step updates particle positions using noisy control inputs.
Noisy Inputs and Measurements:

The code generates noisy control inputs and noisy range measurements for the simulation.
Particle Filter Update:

If a sensor update occurs (every 3 time steps), the code performs the particle filter update.
Particle weights are updated based on the Gaussian likelihood of measured distances.
Resampling is performed if the effective particle count falls below a threshold.
Visualization:

The code visualizes the robot's motion, ground truth trajectory, particle filter estimate, particles, and range measurements using animations.
The animation shows how the particle filter estimate evolves over time, taking measurements into account.
This code demonstrates the use of a Particle Filter with known correspondences to estimate the state of a mobile robot in an environment with RFID tags. It illustrates how the particle filter updates the estimate using noisy sensor measurements and motion dynamics.