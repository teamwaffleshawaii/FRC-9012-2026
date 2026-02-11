// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Joystick Controller;

  // public IntakeSubsystem intakeSubsystem;
  // private LauncherSubsystem launcherSubsystem;
   Compressor phCompressor = new Compressor(2, PneumaticsModuleType.REVPH);
   public  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2,PneumaticsModuleType.REVPH, 0, 1);
    
   //Button on Controller	getRawButton() number
// A	1
// B	2
// X	3
// Y	4
// LB	5
// RB	6
// Back	7
// Start	8



  int intakeExtend = 5;
  int intakeRetract = 6;

  double intakePower = 0.1;
  double transferPower = 0.1;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    phCompressor.enableDigital();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
     //intakeSubsystem.retractIntake();
    
 
   

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  
}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
 //intakeSubsystem.intakeSolenoid.set(Value.kForward);

// A	1
// B	2
// X	3
// Y	4
// LB	5
// RB	6
// Back	7
// Start	8

  // if (Controller.getRawButton(1)) {
  //   intakeSubsystem.extendIntake();
  // }
//TODO check to make sure that running this will not break motors ex have run at opposite sides
// if (Controller.getRawButton(3)) {
 //  launcherSubsystem.runLauncher();
  // }
   if (Controller.getRawButton(2)) {
 intakeSolenoid.set(Value.kForward);
 }
    



//   if (Controller.getRawButton(intakeExtend)) {
//     intakeSubsystem.extendIntake();
//    }
//   else if (Controller.getRawButton(intakeRetract)) {
//     intakeSubsystem.retractIntake();
//    }

  
//   if (Controller.getRawButton(3)) {
//     intakeSubsystem.runIntake(intakePower);
//    }  

//   else if (Controller.getRawButton(2)) {
//       intakeSubsystem.Transfer(transferPower);
//     }

//    else {
//     intakeSubsystem.stopIntake();
//     }
// }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
