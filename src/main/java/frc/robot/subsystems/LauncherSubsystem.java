package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class LauncherSubsystem extends SubsystemBase {

  // Motor Definitions
  private final SparkFlex launcherMotor1 = new SparkFlex(15, MotorType.kBrushless);
  private final SparkFlex launcherMotor2 = new SparkFlex(16, MotorType.kBrushless);

  // Config objects (Matches your Robot.java style)
  public SparkFlexConfig motor1Config = new SparkFlexConfig();
  public SparkFlexConfig motor2Config = new SparkFlexConfig();

  // Closed Loop Controllers
  private final SparkClosedLoopController closedLoopController1 = launcherMotor1.getClosedLoopController();
  private final SparkClosedLoopController closedLoopController2 = launcherMotor2.getClosedLoopController();

  private double cachedVelocity = 0.0;

  public LauncherSubsystem() {
    // Motor 1 Configuration
    motor1Config.inverted(false);
    // Setting PID gains for velocity control
    motor1Config.closedLoop.p(0.0001)
                          .velocityFF(0.00017); 
    
    // Motor 2 Configuration (Churro shaft → Inverted)
    motor2Config.inverted(true);
    motor2Config.closedLoop.p(0.0001)
                          .velocityFF(0.00017);

    // Apply configurations (Matches the safety reset/persist mode)
    launcherMotor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherMotor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    stopLauncher();
  }

  /**
   * Runs the launcher using Velocity Closed Loop
   * @param rpm Target Rotations Per Minute
   */
  public void runLauncher(double rpm) {
    cachedVelocity = rpm;
    // Uses setReference with kVelocity logic from your Robot.java
    closedLoopController1.setReference(rpm, ControlType.kVelocity);
    closedLoopController2.setReference(rpm, ControlType.kVelocity);
  }

  public void stopLauncher() {
    cachedVelocity = 0;
    launcherMotor1.stopMotor();
    launcherMotor2.stopMotor();
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
    launcherMotor1.set(cahcedPower);
    launcherMotor2.set(cahcedPower);
  }
   public void setPower(double power) {
    cahcedPower = power;
    launcherMotor1.set(0);
    launcherMotor2.set(0);
  }
}*/