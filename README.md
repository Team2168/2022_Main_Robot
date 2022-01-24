# 2022_Main_Robot

![image](https://user-images.githubusercontent.com/1295877/149954913-ec88475f-dc2d-41a0-8f8a-b6b7891c93fb.png)

## Subsystems:

* Shooter Wheel
  * 2x TalonFX
* Shooter Hood
  * 1x TalonFX
  * 1x TalonFX limitswitch for home position / zero
* Limelight
  * 1x Limelight
* Turret
  * 1x TalonFX
  * 1x TalonFX limitswitch for home position / zero
* Intake roller
  * 1x TalonFX
* Intake raise/lower
  * 1x Double Solenoid
* Indexer - ball motion preceeding the shooter
  * 1x TalonFX
  * 1x Digital Input for ball detection (IR sensor)
* Hopper - ball motion between the intake and indexer (in the pooper area)
  * 1x TalonFX
  * 1x Digital Input for ball detection (IR sensor)
* Hopper blocker
  * 1x Double Solenoid - prevents the ball from moving into the indexer to aid in alignment for pooper
* Pooper
  * 1x Double Solenoid to eject ball
* Pixy
  * Pixy2 sensor for ball color detection
* Climber
  * 2x TalonFX
  * 1x TalonFX limitswitch for home position / zero
  * 1x DigitalInput for bar detection
* MonkeyBar
  * 1x Double Solenoid for pivot
  * 1x DigitalInput for bar detection
* Drivetrain
  * 6x TalonFX
  * 1x Pigeon IMU

## Guidelines for contributors

See [Contributing.md](/CONTRIBUTING.md)
