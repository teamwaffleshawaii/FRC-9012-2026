package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import java.util.Arrays;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class LauncherSubsystem extends SubsystemBase {

  // Motor Definitions
  private final SparkFlex launcherMotorL = new SparkFlex(15, MotorType.kBrushless);
  private final SparkFlex launcherMotorR = new SparkFlex(16, MotorType.kBrushless);

  private double cachedPower = 0.0;
  private double m_calculatedPower = 0.0;

  public static final int[] validTags = {32,10,11,7};

  public LauncherSubsystem() {

    // Churro shaft → one motor inverted
    launcherMotorL.setInverted(false);
    launcherMotorR.setInverted(true);

    stopLauncher();
  }

  @Override
  public void periodic() {

      boolean hasTarget = LimelightHelpers.getTV("limelight-launch");

      int currentTargetID = (int) LimelightHelpers.getFiducialID("limelight-launch");

      boolean isValidTag = Arrays.stream(validTags).anyMatch(id -> id == currentTargetID);

      double tz = 0.0;

      if (hasTarget && isValidTag) {

          double[] botPose = LimelightHelpers.getBotPose_TargetSpace("limelight-launch");
          tz = Math.abs(botPose[0]);

          double tzMin = .65;
          double tzMax = 1.8;

          double powerMin = 0.85;   // minimum shooter speed
          double powerMax = 1;    // full speed
         
          double clampedTz = MathUtil.clamp(tz, tzMin, tzMax);

          m_calculatedPower =
              powerMin  + (clampedTz - tzMin) * (powerMax - powerMin) / (tzMax - tzMin);

      }

      double launcherVelocityL = launcherMotorL.getEncoder().getVelocity();
      double launcherVelocityR = launcherMotorR.getEncoder().getVelocity();

      SmartDashboard.putNumber("Current Tag ID", currentTargetID);
      SmartDashboard.putBoolean("Valid Tag Locked", isValidTag);
      SmartDashboard.putNumber("Shooter Distance (m)", tz);
      SmartDashboard.putNumber("Shooter Target Power", m_calculatedPower);
      SmartDashboard.putNumber("Actual L RPM", launcherVelocityL);
      SmartDashboard.putNumber("Actual R RPM", launcherVelocityR);
  }

  public double getCalculatedPower() {
      return m_calculatedPower;
  }

  /**
   * Runs launcher with percentage power
   */
  public void runLauncher(double power) {
      cachedPower = power;

      launcherMotorL.set(power);
      launcherMotorR.set(power);
  }

  public void stopLauncher() {
      cachedPower = 0;
      launcherMotorL.stopMotor();
      launcherMotorR.stopMotor();
  }

   public void LauncherOut() {
      

      launcherMotorL.set(-.3);
      launcherMotorR.set(-.3);
  }

   public void launcherBackup() {
      

      launcherMotorL.set(.7);
      launcherMotorR.set(.7);
  }

  public void holdLastPower() {
      runLauncher(cachedPower);
  }

  public void setPower(double power) {
      cachedPower = power;
  }
}