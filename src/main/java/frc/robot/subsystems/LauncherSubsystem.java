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

  //  PID VALUES (tunable from Shuffleboard)
  private double kP = 0.0003; //was .00003
  private double kI = 0.0;
  private double kD = 0.0035; // was .0028
  private double kFF = 0.00195;

  // Track previous values (prevents CAN spam)
  private double prevP = kP;
  private double prevI = kI;
  private double prevD = kD;
  private double prevFF = kFF;

  public LauncherSubsystem() {

    SparkFlexConfig motorConfigL = new SparkFlexConfig();
    SparkFlexConfig motorConfigR = new SparkFlexConfig();

    // Initial config (will be overridden by Shuffleboard)
    motorConfigL.closedLoop
        .p(kP)
        .d(kD)
        .velocityFF(kFF);

    motorConfigL
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(60);

    // Motor R follows L (inverted)
    motorConfigR
        .follow(launcherMotorL, true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(60);

    launcherMotorL.configure(motorConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherMotorR.configure(motorConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize Shuffleboard values
    //SmartDashboard.putNumber("Shooter/Target RPM", m_calculatedRPM);
     SmartDashboard.putNumber("Shooter/Actual L RPM", launcherMotorL.getEncoder().getVelocity());
     SmartDashboard.putNumber("Shooter/Actual R RPM", launcherMotorR.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter/P", kP);
    SmartDashboard.putNumber("Shooter/D", kD);
    SmartDashboard.putNumber("Shooter/FF", kFF);
  
    

    stopLauncher();
  }

  @Override
  public void periodic() {

    // ================= LIMELIGHT RPM CALC =================
    boolean hasTarget = LimelightHelpers.getTV("limelight-launch");
    int currentTargetID = (int) LimelightHelpers.getFiducialID("limelight-launch");
    boolean isValidTag = Arrays.stream(validTags).anyMatch(id -> id == currentTargetID);

    if (hasTarget && isValidTag) {
        double[] botPose = LimelightHelpers.getBotPose_TargetSpace("limelight-launch");
        double tz = Math.abs(botPose[2]);

        double tzMin = 1.1; 
        double tzMax = 2.2;
        double rpmMin = 4333; 
        double rpmMax = 6784.0; 

        double clampedTz = MathUtil.clamp(tz, tzMin, tzMax);
        m_calculatedRPM = rpmMin + (clampedTz - tzMin) * (rpmMax - rpmMin) / (tzMax - tzMin);
    }

    // ================= READ PID FROM SHUFFLEBOARD =================
    kP = SmartDashboard.getNumber("Shooter/P", kP);
    kI = SmartDashboard.getNumber("Shooter/I", kI);
    kD = SmartDashboard.getNumber("Shooter/D", kD);
    kFF = SmartDashboard.getNumber("Shooter/FF", kFF);

    
double shootingrpm = 4750;
    // ================= DASHBOARD OUTPUT =================
    SmartDashboard.putNumber("Shooter/Target RPM", shootingrpm);
    SmartDashboard.putNumber("Shooter/Actual L RPM", launcherMotorL.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter/Actual R RPM", launcherMotorR.getEncoder().getVelocity());
  }

  // ================= CONTROL METHODS =================

  public void runVelocityFromLimelight() {
    closedLoopControllerL.setReference(m_calculatedRPM, ControlType.kVelocity);
  }

  public void runFixedShooting( ) {
    closedLoopControllerL.setReference(4650, ControlType.kVelocity);
  }

  public void runFromDashboard() {
    double setpoint = SmartDashboard.getNumber("Shooter/Setpoint", 0);
    closedLoopControllerL.setReference(setpoint, ControlType.kVelocity);
  }

  public void runLauncher(double rpm) {
    closedLoopControllerL.setReference(rpm, ControlType.kVelocity);
    
  }

  public double getCalculatedRPM() {
    return m_calculatedRPM;
}

  public void stopLauncher() {
    launcherMotorL.stopMotor();
  }

}