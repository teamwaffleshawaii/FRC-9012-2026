package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VortexConstants;

public class LauncherSubsystem extends SubsystemBase {

    private final SparkFlex LauncherMotor1 =
        new SparkFlex(VortexConstants.kLeaderID,MotorType.kBrushless);
 
    private final SparkFlex LauncherMotor2 =
        new SparkFlex(VortexConstants.kFollowerID, MotorType.kBrushless);

    public double speed = 0.1;
// try first motor before running both at once
    /** Run motors at percent output (-1.0 to 1.0) */
    public void runLauncher(double speed) {
        LauncherMotor1.set(speed);
        LauncherMotor2.set(-speed);
    } 

    /** Stop motors */
    public void stopLauncher() {
        LauncherMotor1.stopMotor();
        LauncherMotor2.stopMotor();
    }

    @Override
    public void periodic() {
        // Optional: telemetry later
    }
}
