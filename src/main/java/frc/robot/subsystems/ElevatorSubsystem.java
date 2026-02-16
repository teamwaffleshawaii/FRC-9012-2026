package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax leftMotor = new SparkMax(11, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(12, MotorType.kBrushless);

    private static final double kUpPosition = 35.5;
    private static final double kDownPosition = 0.0;

    public ElevatorSubsystem() {

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);

        leftMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters,
                    SparkMax.PersistMode.kPersistParameters);

rightMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters,
                     SparkMax.PersistMode.kPersistParameters);
        // Invert ONE motor
        rightMotor.setInverted(true);

        // PID values (START HERE — you will tune later)
        config.closedLoop.pid(0.1, 0.0, 0.01);
    }

    public void goToPosition(double position) {
        leftMotor.getClosedLoopController()
                .setReference(position, SparkMax.ControlType.kPosition);

        rightMotor.getClosedLoopController()
                .setReference(position, SparkMax.ControlType.kPosition);
    }

    public void goUp() {
        goToPosition(kUpPosition);
    }

    public void goDown() {
        goToPosition(kDownPosition);
    }

    public double getPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putNumber("Elevator Left Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Right Current", rightMotor.getOutputCurrent());
    }
}