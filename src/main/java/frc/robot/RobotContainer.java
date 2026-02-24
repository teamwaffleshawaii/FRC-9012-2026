// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToAprilTagCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.TransferSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private final TransferSubsystem m_Transfer = new TransferSubsystem();
   private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  //...Add more here

  
  //Controller for Driver 1 (Joystick)
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  //Controller for Driver 2 (Custom Controller)
  private final GenericHID operatorController = new GenericHID(1); //if using custom controller

  //For drivetrain speed control
  double speedFactor = 0.5 * (1 - m_driverController.getRawAxis(4));  //drivetrain speed control 
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
     NamedCommands.registerCommand(
            "AlignToTag31",
            new AlignToAprilTagCommand(
                m_robotDrive,
                9,     // AprilTag ID
                3.0    // timeout
            )
        );
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands, updated with speed
    m_robotDrive.setDefaultCommand(
    new RunCommand(
        () -> {
            // ---------- Translation Speed Factor (Axis 3) ----------
            double rawThrottle = -m_driverController.getRawAxis(3);  // axis for throttle

            double translationFactor = MathUtil.applyDeadband(rawThrottle, 0.05);
            translationFactor = (translationFactor + 1) / 2.0;       // -1..1 -> 0..1
            translationFactor = MathUtil.clamp(translationFactor, 0.2, 1.0); // min 20% speed

            // ---------- Rotation Speed Factor (Axis 2) ----------
            // You could also tie this to a separate slider or keep it fixed
            double rawRot = -m_driverController.getRawAxis(2); // Right stick X
            double rotationFactor = MathUtil.applyDeadband(rawRot, 0.05);
            rotationFactor = rotationFactor * 0.5; // limit max rotation speed to 50% of full

            // ---------- Driver Joystick Inputs ----------
            double xSpeed = -MathUtil.applyDeadband(m_driverController.getRawAxis(1), OIConstants.kDriveDeadband) * translationFactor;
            double ySpeed = -MathUtil.applyDeadband(m_driverController.getRawAxis(0), OIConstants.kDriveDeadband) * translationFactor;
            double rotSpeed = rotationFactor; // rotation already scaled separately

            // ---------- Drive Command ----------
            m_robotDrive.drive(xSpeed, ySpeed, rotSpeed, true);

            // ---------- Telemetry ----------
            SmartDashboard.putNumber("Translation Factor", translationFactor);
            SmartDashboard.putNumber("Rotation Factor", rotationFactor);
        },
        m_robotDrive
    )
);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 3)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, 2)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    //Add button bindings here:

    //Button 1 → Intake Pivot UP
    new JoystickButton(operatorController, 1)
      .onTrue(new InstantCommand(m_intake::pivotUp, m_intake));

    //Button 2 → Intake Pivot DOWN
    new JoystickButton(operatorController, 2)
      .onTrue(new InstantCommand(m_intake::pivotDown, m_intake));

    //Button 3 → Intake
    new JoystickButton(operatorController, 3)
      .onTrue(new InstantCommand(m_intake::intakeIn, m_intake))
     .onTrue(new InstantCommand(m_Transfer::transferInSlow, m_Transfer))
     .onTrue(new InstantCommand(m_Transfer::mecanumOut, m_Transfer))
      .onFalse(new InstantCommand(() -> m_intake.intakeStop(), m_intake))
      .onFalse(new InstantCommand(m_Transfer::transferStop, m_Transfer))
     .onFalse(new InstantCommand(m_Transfer::mecanumStop, m_Transfer));

    //Button 4 → Elevator up MAX
    new JoystickButton(operatorController, 4)
      .onTrue(new InstantCommand(m_elevator::goTop, m_elevator));

    //Button 5 → Elevator down MIN
    new JoystickButton(operatorController, 5)
      .onTrue(new InstantCommand(m_elevator::goBottom, m_elevator));
      
    //Button 6 → Elevator up manual
    new JoystickButton(operatorController, 6)
      .whileTrue(new RunCommand(() -> 
      m_elevator.adjustPosition(0.2), m_elevator));

    //Button 7 → Elevator down manual
    new JoystickButton(operatorController, 7)
      .whileTrue(new RunCommand(() -> 
      m_elevator.adjustPosition(-0.2), m_elevator));

    
    //Button 8 & Button 9 pending Apriltags

    //Button 10 → Launch FUELs (Transfer on)
    new JoystickButton(operatorController, 10)
    .onTrue(new InstantCommand(m_Transfer::transferIn, m_Transfer))
    .onTrue(new InstantCommand(m_Transfer::mecanumIn, m_Transfer))
    .onTrue(new InstantCommand(m_intake::intakeTransfer, m_intake))
    .onFalse(new InstantCommand(() -> m_Transfer.transferStop(), m_Transfer))
    .onFalse(new InstantCommand(() -> m_Transfer.mecanumStop(), m_Transfer))
    .onFalse(new InstantCommand(() -> m_intake.intakeStop(), m_intake));
    
    //Button 11 → Launchers On
    new JoystickButton(operatorController, 11)
    .onTrue(new InstantCommand(() -> m_launcher.runLauncher(0.7), m_launcher));

    //Button 12 → Launchers Off
    new JoystickButton(operatorController, 12)
    .onTrue(new InstantCommand(() -> m_launcher.stopLauncher(), m_launcher));


     // Constants
    double targetDistanceMetersTZ = 1.40; // meters
    double targetDistanceMetersTX = 0.45; // meters
    double targetHeadingRY = 0;

    double distanceKp = 0.5;
    double strafeKp = 0.5;
    double steeringKp = 0.5;

    // >>> SELECT WHICH APRILTAG TO ALIGN TO <<<
    int targetAprilTagID = 9;

 new JoystickButton(operatorController, 8)
        .whileTrue(new RunCommand(
            () -> {

                double seenTagID = LimelightHelpers.getFiducialID("limelight");

                if (seenTagID != targetAprilTagID) {
                    m_robotDrive.drive(0, 0, 0, false);
                    return;
                }

                // botPose array: [x, y, z, roll, pitch, yaw]
                double[] botPose = LimelightHelpers.getBotPose_TargetSpace("limelight");

                double currentStrafeX  = botPose[0]; // Left/Right
                double currentDistanceZ = botPose[2]; // Forward/Back
                double currentYaw       = botPose[4]; // Rotation relative to AprilTag

                // Errors
                double distanceError = -targetDistanceMetersTZ - currentDistanceZ;
                double strafeError   = -targetDistanceMetersTX - currentStrafeX;
                double steeringError = targetHeadingRY - currentYaw;
                double radError = Math.toRadians(steeringError);

                // Deadbands
                if (Math.abs(distanceError) < 0.02) distanceError = 0;
                if (Math.abs(strafeError) < 0.02) strafeError = 0;
                if (Math.abs(steeringError) < 2.0) steeringError = 0;

                m_robotDrive.drive(
                    distanceError * distanceKp,   // Forward/back
                    -strafeError * strafeKp,      // Left/Right
                    -radError * steeringKp,       // Rotation
                    false
                );
            },
            m_robotDrive
        ));

    int[] VALID_TAGS = {9};



    new JoystickButton(operatorController, 9).whileTrue(new RunCommand(() -> {

    boolean hasTarget = LimelightHelpers.getTV("limelight");

    if (!hasTarget) {
        m_launcher.stopLauncher();
        return;
    }

    double[] botPose = LimelightHelpers.getBotPose_TargetSpace("limelight");
    double tz = Math.abs(botPose[2]);

    // ----- Tunable Constants -----
    double tzMin = 0.8;   // ~2.5 ft
    double tzMax = 3.0;   // ~10 ft
    double powerMin = 0.30;
    double powerMax = 0.65;

    double clampedTz = MathUtil.clamp(tz, tzMin, tzMax);

    double motorPower =
            powerMin +
            (clampedTz - tzMin) * (powerMax - powerMin) / (tzMax - tzMin);

    m_launcher.setPower(motorPower);

    SmartDashboard.putNumber("Shooter Distance (m)", tz);
    SmartDashboard.putNumber("Shooter Power", motorPower);

}, m_launcher));}
                

    //Data collected from February 12th
    //86" away, 65 percent
    //103" away, 70 percent


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 

  private double deadband(double value) {
    return Math.abs(value) > 0.08 ? value : 0.0;
}

  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
