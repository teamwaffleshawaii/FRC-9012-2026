# FRC 9012 Team Waffles

* Pneumatic Hub (CAN ID 2)
* MaxSwerve using NEO Vortex and NEO 550 (CAN IDs 3,4,5,6,7,8,9,10)
* 2 Elevator NEO motors (CAN IDs 11, 12)
* 2 Intake NEO 550 motors (CAN IDs 13, 14)
* 2 Launchers NEO Vortex (CAN IDs 15, 16)
* 1 Transfer NEO550 motor polycarbonate tubes (CAN ID 17)
* 1 Transfer NEO 550 motor mecanum rollers (CAN ID 18)

## MaxSwerve Prerequisites

* SPARK MAX Firmware v26.1.0
* REVLib v2026.0.0

## MaxSwerve Configs & Constants

* Drivetrain Gear Max Speed: 7.22 m/s
* TrackWidth: 22.5 inches
* DrivingMotor Pinion Teeth: 16T
* SpurGear Tetth: 20T
* NEO Vortex FreeSpeed RPM: 6784

It is possible that this project will not work for your robot right out of the box. Various things like the CAN IDs, PIDF gains, chassis configuration, etc. must be determined for your own robot!

These values can be adjusted in the `Configs.java` and `Constants.java` files.
