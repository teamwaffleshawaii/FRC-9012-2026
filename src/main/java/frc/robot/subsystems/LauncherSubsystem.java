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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class LauncherSubsystem extends SubsystemBase {

  private final SparkFlex launcherMotorL = new SparkFlex(15, MotorType.kBrushless);
  private final SparkFlex launcherMotorR = new SparkFlex(16, MotorType.kBrushless);

  private final SparkClosedLoopController closedLoopControllerL = launcherMotorL.getClosedLoopController();

  private double m_calculatedRPM = 0.0;
  public static final int[] validTags = {9, 10, 25, 26};

  public LauncherSubsystem() {
    SparkFlexConfig motorConfigL = new SparkFlexConfig();
    SparkFlexConfig motorConfigR = new SparkFlexConfig();

   // p org was 0.005
   // ff was org 0.000147
    motorConfigL.closedLoop
        .p(0.0001) 
        .d(0.0001)
        .velocityFF(0.0002);
    
    motorConfigL
        .inverted(false)
        .idleMode(IdleMode.kCoast) 
        .smartCurrentLimit(60);

    // Motor R Configuration: Follows Motor L but inverted
    motorConfigR
        .follow(launcherMotorL, true) // 'true' inverts the follower
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(60);

    // Apply configurations
    launcherMotorL.configure(motorConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherMotorR.configure(motorConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    stopLauncher();
  }

  @Override
  public void periodic() {
    boolean hasTarget = LimelightHelpers.getTV("limelight-launch");
    int currentTargetID = (int) LimelightHelpers.getFiducialID("limelight-launch");
    boolean isValidTag = Arrays.stream(validTags).anyMatch(id -> id == currentTargetID);

    if (hasTarget && isValidTag) {
        double[] botPose = LimelightHelpers.getBotPose_TargetSpace("limelight-launch");
        double tz = Math.abs(botPose[2]);

      
        double tzMin = .8; 
        double tzMax = 3.0;
        double rpmMin = 4000; 
        double rpmMax = 6784.0; 

        double clampedTz = MathUtil.clamp(tz, tzMin, tzMax);
        m_calculatedRPM = rpmMin + (clampedTz - tzMin) * (rpmMax - rpmMin) / (tzMax - tzMin);
    } 

    // Dashboard updates
    SmartDashboard.putNumber("Shooter/Target RPM", m_calculatedRPM);
    SmartDashboard.putNumber("Shooter/Actual L RPM", launcherMotorL.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter/Actual R RPM", launcherMotorR.getEncoder().getVelocity());
  }


  public void runVelocityFromLimelight() {
    closedLoopControllerL.setReference(m_calculatedRPM, ControlType.kVelocity);
  }


  public void runFixedShooting() {
    closedLoopControllerL.setReference(4800.0, ControlType.kVelocity);
  }


  public void runLauncher(double rpm) {
    closedLoopControllerL.setReference(rpm, ControlType.kVelocity);
  }

  public void stopLauncher() {
    closedLoopControllerL.setReference(0, ControlType.kVelocity);
  }
}