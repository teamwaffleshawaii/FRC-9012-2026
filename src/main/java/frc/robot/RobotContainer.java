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
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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
  private final IntakeSubsystem intake = new IntakeSubsystem();
  //...Add more here
  
  //Controller for Driver 1
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  //Controller for Driver 2
  private final XboxController operatorController = new XboxController(1);

  //For drivetrain speed control
  double speedFactor = 0.5 * (1 - m_driverController.getRawAxis(4));  //drivetrain speed control 
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands, updated with speed
    m_robotDrive.setDefaultCommand(
    new RunCommand(
        () -> {
            // ---------- Translation Speed Factor (Axis 4) ----------
            double rawThrottle = -m_driverController.getRawAxis(4); // Right stick Y
            double translationFactor = MathUtil.applyDeadband(rawThrottle, 0.05);
            translationFactor = (translationFactor + 1) / 2.0;       // -1..1 -> 0..1
            translationFactor = MathUtil.clamp(translationFactor, 0.2, 1.0); // min 20% speed

            // ---------- Rotation Speed Factor (Axis 3) ----------
            // You could also tie this to a separate slider or keep it fixed
            double rawRot = -m_driverController.getRawAxis(3); // Right stick X
            double rotationFactor = MathUtil.applyDeadband(rawRot, 0.05);
            rotationFactor = rotationFactor * 0.5; // limit max rotation speed to 50% of full

            // ---------- Driver Joystick Inputs ----------
            double xSpeed = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * translationFactor;
            double ySpeed = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * translationFactor;
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
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    //Add more button bindings here:
    //...
    //...
    //So far, this includes intake only

    // A → Intake ON
    new JoystickButton(operatorController, XboxController.Button.kA.value)
      .onTrue(new InstantCommand(intake::intakeIn, intake));

    // B → Intake OFF
    new JoystickButton(operatorController, XboxController.Button.kB.value)
      .onTrue(new InstantCommand(intake::stopIntake, intake));

    // D-pad UP → Pivot UP
    new POVButton(operatorController, 0)
      .onTrue(new InstantCommand(intake::pivotUp, intake));

    // D-pad DOWN → Pivot DOWN
    new POVButton(operatorController, 180)
      .onTrue(new InstantCommand(intake::pivotDown, intake));
  }

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
