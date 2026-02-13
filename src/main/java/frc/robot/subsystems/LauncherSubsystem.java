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

  public void runLauncher(double speed) {
    launcherMotor1.set(speed);
    launcherMotor2.set(speed);
  }

  public void stopLauncher() {
    launcherMotor1.set(0);
    launcherMotor2.set(0);
  }
}