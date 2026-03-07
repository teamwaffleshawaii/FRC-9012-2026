package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import java.util.Arrays;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class LauncherSubsystem extends SubsystemBase {

  // Motor Definitions
  private final SparkFlex launcherMotorL = new SparkFlex(15, MotorType.kBrushless);
  private final SparkFlex launcherMotorR = new SparkFlex(16, MotorType.kBrushless);

  // Config objects (Matches your Robot.java style)
  public SparkFlexConfig motorConfigL = new SparkFlexConfig();
  public SparkFlexConfig motorConfigR = new SparkFlexConfig();

  // Closed Loop Controllers
  private final SparkClosedLoopController closedLoopControllerL = launcherMotorL.getClosedLoopController();
  private final SparkClosedLoopController closedLoopControllerR = launcherMotorR.getClosedLoopController();

  private double cachedVelocity = 0.0;
  private double m_calculatedRPM = 0.0;

  public static final int[] validTags = {32,10};

  public LauncherSubsystem() {
    // Motor 1 Configuration
    motorConfigL.inverted(false);
    // Setting PID gains for velocity control
    motorConfigL.closedLoop.p(0.005) // Reduced from 0.01
                        .d(0.0001) // Added a tiny bit of damping
                        .velocityFF(0.000147);
    
    // Motor 2 Configuration (Churro shaft → Inverted)
    motorConfigR.inverted(true);
    motorConfigR.closedLoop.p(0.005) // Reduced from 0.01
                        .d(0.0001) // Added a tiny bit of damping
                        .velocityFF(0.000147);

    // Apply configurations (Matches the safety reset/persist mode)
    launcherMotorL.configure(motorConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherMotorR.configure(motorConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    stopLauncher();
  }

  // Inside Launcher.java
  @Override
  public void periodic() {
      boolean hasTarget = LimelightHelpers.getTV("limelight-launch");
      
      // 1. Get the ID of the current target
      int currentTargetID = (int) LimelightHelpers.getFiducialID("limelight-launch");

      // 2. List of valid tags up at top of code

      // 3. Checks if the targetID = any of the list of valid tags
      boolean isValidTag = Arrays.stream(validTags).anyMatch(id -> id == currentTargetID);

      double tz = 0.0;

      // 4. Only run the math if we HAVE a target AND it's a VALID tag
      if (hasTarget && isValidTag) {
          double[] botPose = LimelightHelpers.getBotPose_TargetSpace("limelight-launch");
          tz = Math.abs(botPose[2]);

          double tzMin = 1.2; double tzMax = 3.0;
          double rpmMin = 0.0; double rpmMax = 6784.0; // CHECK REV HARDWARE CLIENT MOTOR RPM (VELOCITY)

          double clampedTz = MathUtil.clamp(tz, tzMin, tzMax);
          m_calculatedRPM = rpmMin + (clampedTz - tzMin) * (rpmMax - rpmMin) / (tzMax - tzMin);
      } 

      double launcherVelocityL = launcherMotorL.getEncoder().getVelocity();
      double launcherVelocityR = launcherMotorR.getEncoder().getVelocity();


      // Dashboard updates
      SmartDashboard.putNumber("Current Tag ID", currentTargetID);
      SmartDashboard.putBoolean("Valid Tag Locked", isValidTag);
      SmartDashboard.putNumber("Shooter Distance (m)", tz);
      SmartDashboard.putNumber("Shooter Target RPM", m_calculatedRPM);
      SmartDashboard.putNumber("Actual L RPM", launcherVelocityL);
      SmartDashboard.putNumber("Actual R RPM", launcherVelocityR);
  }

  public double getCalculatedRPM() {
        return m_calculatedRPM;
    }

  /**
   * Runs the launcher using Velocity Closed Loop
   * @param rpm Target Rotations Per Minute
   */
  public void runLauncher(double rpm) {
    cachedVelocity = rpm;
    // Uses setReference with kVelocity logic from your Robot.java
    closedLoopControllerL.setReference(rpm, ControlType.kVelocity);
    closedLoopControllerR.setReference(rpm, ControlType.kVelocity);
  }

  public void runLauncherPower(double speed) {
    launcherMotorL.set(speed);
    launcherMotorR.set(speed);
  }

  public void stopLauncher() {
    cachedVelocity = 0;
    launcherMotorL.stopMotor();
    launcherMotorR.stopMotor();
  }

  public void holdLastVelocity() {
    runLauncher(cachedVelocity);
  }

  public void setTargetVelocity(double rpm) {
    cachedVelocity = rpm;
  }
}


/* 

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class LauncherSubsystem extends SubsystemBase {

  private final SparkFlex launcherMotor1 =
    new SparkFlex(15, MotorType.kBrushless);

  private final SparkFlex launcherMotor2 =
    new SparkFlex(16, MotorType.kBrushless);

  public LauncherSubsystem() {

    // Churro shaft → one motor must be inverted
    launcherMotor2.setInverted(true);

    stopLauncher();
  }

  public double speed = 0.75;
  public double cahcedPower = 0.0;

  public void runLauncher(double speed) {
    launcherMotor1.set(speed);
    launcherMotor2.set(speed);
  }

  public void stopLauncher() {
    launcherMotor1.set(0);
    launcherMotor2.set(0);
  }
  public void holdLastPower() {
    launcherMotor1.set(cachedPower);
    launcherMotor2.set(cachedPower);
  }
   public void setPower(double power) {
    cachedPower = power;
    launcherMotor1.set(0);
    launcherMotor2.set(0);
  }
}*/




