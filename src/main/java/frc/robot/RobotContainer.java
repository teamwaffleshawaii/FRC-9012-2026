// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToAprilTagCommand;
import frc.robot.commands.AlingToTowerCommand;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    
  private final ProfiledPIDController rotationPID = new ProfiledPIDController(
    10.0, 0.0, 0.0, AutoConstants.kThetaControllerConstraints
  );


  // The robot's subsystems
   final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private final TransferSubsystem m_transfer = new TransferSubsystem();
  private final LEDSubsystem m_leds = new LEDSubsystem();
   private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  //...Add more here

   private final SendableChooser<Command> autoChooser;

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

    rotationPID.enableContinuousInput(-180, 180); // crucial for full rotations
    rotationPID.setTolerance(1.0);           // optional, small deadband

        // Register Named Commands (EXAMPLES)
        // NamedCommands.registerCommand("Auto", DriveSubsystem.autoBalanceCommand());
        // NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
        // NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());

    NamedCommands.registerCommand(
            "AlignToTag10",
            new AlignToAprilTagCommand(
                m_robotDrive,
                6,   // AprilTag ID
                3.0    // timeout
            )   
    );
 
    NamedCommands.registerCommand("IntakeDown",
        new SequentialCommandGroup(new InstantCommand(m_intake::pivotDown, m_intake)));
    
    // Action: Intake Up/Store (Button 1 logic)
    NamedCommands.registerCommand("IntakeUp", new ParallelCommandGroup(
       // new StartEndCommand(m_intake::intakeInFullPower, m_intake::intakeStop).withTimeout(3.0),
        new SequentialCommandGroup(new InstantCommand(m_intake::pivotUp))));

        NamedCommands.registerCommand("IntakeIn", new ParallelCommandGroup(
        new StartEndCommand(m_intake::intakeIn, m_intake::intakeStop).withTimeout(3.0)));
       


    // Action: Spin up and Launch (Button 11 + 10 logic)
         NamedCommands.registerCommand("Launcher", new ParallelCommandGroup(
        new ParallelCommandGroup(
          new InstantCommand(m_launcher::runFixedShooting, m_launcher),
            new InstantCommand(m_transfer::transferIn, m_transfer),
            new InstantCommand(m_transfer::mecanumIn, m_transfer),
            new InstantCommand(m_intake::intakeTransfer, m_intake)
        )));
        // NamedCommands.registerCommand("LaunchSequence", new SequentialCommandGroup(
    

    // // Action: Stop all intake/launcher motors
    NamedCommands.registerCommand("StopAll", new ParallelCommandGroup(
        new InstantCommand(m_launcher::stopLauncher, m_launcher),
        new InstantCommand(m_transfer::transferStop, m_transfer),
        new InstantCommand(m_intake::intakeStop, m_intake),
        new InstantCommand(m_transfer::mecanumStop, m_transfer)
    ));

    // Do all other initialization
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    // register NamedCommands FIRST

   
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the button bindings
    configureButtonBindings();
   m_leds.setDefaultCommand(new RunCommand(m_leds::rainbow, m_leds));

    
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
          }, m_robotDrive
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
    new JoystickButton(m_driverController, 6)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, 2)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

          //  new JoystickButton(m_driverController, 11)
        // .onTrue(new InstantCommand(
        //     () -> m_launcher.adjustBackupPower( 0.005)));
        //    ;
        //    new JoystickButton(m_driverController, 12)
        // .onTrue(new InstantCommand(
        //     () -> m_launcher.adjustBackupPower(-.005)));
        //    ;



    // >>> SELECT WHICH APRILTAG TO ALIGN TO <<<
    // desired tags 1,6,7,9,10,12,15,22,23,25,26,31
   

   
 new JoystickButton(m_driverController, 1)
    .whileTrue(new AlignToAprilTagCommand(
        m_robotDrive,
        6,      // target tag ID
        3.0     // timeout seconds
    ));

    new JoystickButton(m_driverController, 4)
    .whileTrue(new AlingToTowerCommand(
        m_robotDrive,
        6,      // target tag ID
        3.0     // timeout seconds
    ));


    new JoystickButton(m_driverController, 1)
    .whileTrue(new AlignToAprilTagCommand(
        m_robotDrive,
        6,      // target tag ID
        3.0     // timeout seconds
    ));

    //Add button bindings here:

    //Button 1 → Intake Pivot UP
    new JoystickButton(operatorController, 1)
      .onTrue(new SequentialCommandGroup(new InstantCommand(m_intake::pivotUp, m_intake)));
     // .onTrue(new SequentialCommandGroup(new InstantCommand(m_launcher::launcherBackup, m_launcher)));

    //Button 2 → Intake Pivot DOWN
    new JoystickButton(operatorController, 2)
      .onTrue(new SequentialCommandGroup(new InstantCommand(m_intake::pivotDown, m_intake)));
      
    //Button 3 → Intake
    new JoystickButton(operatorController, 3)
      .onTrue(
        new SequentialCommandGroup(
           new InstantCommand(m_leds::intakeColor, m_leds),  
          new InstantCommand(m_intake::intakeIn, m_intake),            // intake, intake
          new InstantCommand(m_transfer::transferInSlow, m_transfer),  // transfer in
          new InstantCommand(m_transfer::mecanumOut, m_transfer)       // mecanum in
        )
      )
      .onFalse(
        new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.intakeStop(), m_intake), // intake stop
            new InstantCommand(m_transfer::transferStop, m_transfer),  // transfer stop
            new InstantCommand(m_transfer::mecanumStop, m_transfer)    // mecanum stop
        )
      );

    //Button 4 → Elevator up MAX
    new JoystickButton(operatorController, 4)
      .onTrue(new InstantCommand(m_elevator::goTop, m_elevator));

    //Button 5 → Elevator down MIN
    new JoystickButton(operatorController, 5)
      .onTrue(new InstantCommand(m_elevator::goBottom, m_elevator));
      
      double elevatorManualSpeedUp = 0.5;
      double elevatorManualSpeedDown = 0.5; // do not put minus in this
    
    //Button 6 → Elevator up manual
    new JoystickButton(operatorController, 6)
      .whileTrue(new RunCommand(() -> 
      m_elevator.adjustPosition(elevatorManualSpeedUp), m_elevator));

    //Button 7 → Elevator down manual
    new JoystickButton(operatorController, 7)
      .whileTrue(new RunCommand(() -> 
      m_elevator.adjustPosition(-elevatorManualSpeedDown), m_elevator));
    
    //Button 8 → Reverse intake (safety)
    new JoystickButton(operatorController, 8)
      .onTrue(new InstantCommand(m_intake::intakeOut, m_intake));
    
    //Button 9 → Reverse transfer (safety)
    //Yet to be added

    //Button 10 → Launch FUELs (Transfer on)
    new JoystickButton(operatorController, 10)
      .onTrue(new SequentialCommandGroup( // changed from SequentialCommandGroup
        new InstantCommand(m_transfer::transferIn, m_transfer),
        new InstantCommand(m_transfer::mecanumIn, m_transfer),
        new InstantCommand(m_intake::intakeTransfer, m_intake)
      ))
      .onFalse(new SequentialCommandGroup( // changed from SequentialCommandGroup
        new InstantCommand(() -> m_transfer.transferStop(), m_transfer),
        new InstantCommand(() -> m_transfer.mecanumStop(), m_transfer),
        new InstantCommand(() -> m_intake.intakeStop(), m_intake)
      ));
    
    new JoystickButton(operatorController, 11)
      .onTrue(new SequentialCommandGroup( // changed from SequentialCommandGroup
        new InstantCommand(m_launcher::runFixedShooting, m_launcher)));

        // return m_launcher.backupPower
        ;

   new JoystickButton(operatorController, 9)
      .onTrue(new SequentialCommandGroup( // changed from SequentialCommandGroup
        new InstantCommand(m_leds::reverseColor, m_leds),
        new InstantCommand(m_transfer::transferOut, m_transfer),
        new InstantCommand(m_transfer::mecanumOut, m_transfer)
       // new InstantCommand(m_launcher::LauncherOut, m_launcher)
      ))
      .onFalse(new SequentialCommandGroup(
      new InstantCommand(() -> m_transfer.transferStop(), m_transfer),
        new InstantCommand(() -> m_transfer.mecanumStop(), m_transfer)
        //new InstantCommand(() -> m_launcher.stopLauncher(), m_launcher)
      ));
     
      
    

    //Button 12 → Launchers Off
    new JoystickButton(operatorController, 12)
      .onTrue(new InstantCommand(() -> m_launcher.stopLauncher(), m_launcher));

    
/*
     // Constants
    double targetDistanceMetersTZ = 1.40; // meters
    double targetDistanceMetersTX = 0.45; // meters
    double targetHeadingRY = 0;

    double distanceKp = 0.5;
    double strafeKp = 0.5;
    double steeringKp = 0.5;

    // >>> SELECT WHICH APRILTAG TO ALIGN TO <<<
    int targetAprilTagID = 15;

 new JoystickButton(operatorController, 8)
        .whileTrue(new RunCommand(
            () -> {

                double seenTagID = LimelightHelpers.getFiducialID("limelight-launch");

                if (seenTagID != targetAprilTagID) {
                    m_robotDrive.drive(0, 0, 0, false);
                    return;
                }
                int seenID = (int) LimelightHelpers.getFiducialID("limelight-launch");
                if (seenID != 32) {
                    m_launcher.stopLauncher();
                    return;
                }
                // botPose array: [x, y, z, roll, pitch, yaw]
                double[] botPose = LimelightHelpers.getBotPose_TargetSpace("limelight-launch");

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
*/

    // BUTTON NUMBER 9 (right Apr-Tag) sets launcher rpm to distance value of apriltag
    // new JoystickButton(operatorController, 11).onTrue(new RunCommand(() -> {


    //         new InstantCommand(m_launcher::launcherBackup, m_launcher);
    //         // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-launch", LauncherSubsystem.validTags);
    //         // m_launcher.runLauncher(m_launcher.getCalculatedPower());

    //      }, m_launcher))//.onFalse(
    //                     //  new SequentialCommandGroup(
    //                     //      new InstantCommand(m_launcher::holdLastPower, m_launcher),
    //                     //      new InstantCommand(() -> LimelightHelpers.SetFiducialIDFiltersOverride("limelight-launch", new int[] {}))
    //                     // )//)
    //                   ;

/* 
    new JoystickButton(operatorController, 9).whileTrue(new RunCommand(() -> {
        boolean hasTarget = LimelightHelpers.getTV("limelight-launch");
        
        // 1. Declare variables with default values here so the whole command can see them
        double tz = 0.0;
        double targetRPM = 0.0; 

        if (hasTarget) {
            double[] botPose = LimelightHelpers.getBotPose_TargetSpace("limelight-launch");
            tz = Math.abs(botPose[2]); // Update the outer variable

            double tzMin = 0.4;
            double tzMax = 3.0;
            double rpmMin = 0.0;
            double rpmMax = 6784.0;

            double clampedTz = MathUtil.clamp(tz, tzMin, tzMax);

            // 2. Assign the calculation to the variable declared above
            targetRPM = rpmMin + (clampedTz - tzMin) * (rpmMax - rpmMin) / (tzMax - tzMin);

            m_launcher.runLauncher(targetRPM);
        } 
        else {
            m_launcher.holdLastVelocity();
            // Optionally: targetRPM = m_launcher.getLastVelocity(); // To keep dashboard accurate
        }

        // 3. Now these can access 'tz' and 'targetRPM' regardless of the if/else outcome
        SmartDashboard.putNumber("Shooter Distance (m)", tz);
        SmartDashboard.putNumber("Shooter Distance (ft)", (tz * 3.28084));
        SmartDashboard.putNumber("Shooter RPM", targetRPM);

    }, m_launcher)).onFalse(new InstantCommand(m_launcher::stopLauncher, m_launcher)); */
  

  
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
    return autoChooser.getSelected();
    /*
    // Create config for trajectory
    //return new PathPlannerAuto("Example Auto");
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
    */
  }
}
