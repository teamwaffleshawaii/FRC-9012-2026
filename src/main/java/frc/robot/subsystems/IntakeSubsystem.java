package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

    // ==============================
    // Motor Controllers
    // ==============================

    private final SparkMax intakeLeft = new SparkMax(13, MotorType.kBrushless);
    private final SparkMax intakeRight = new SparkMax(14, MotorType.kBrushless);

    // ==============================
    // Pneumatics (REV PH CAN ID: 2)
    // ==============================

    private final DoubleSolenoid pivotSolenoid =
        new DoubleSolenoid(
            2, // REV Pneumatic Hub CAN ID
            PneumaticsModuleType.REVPH,
            0, // forward channel
            1  // reverse channel
        );

    // ==============================
    // Constructor
    // ==============================

    public IntakeSubsystem() {
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        // Inversion
        rightConfig.inverted(true);
        // Brake mode
        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);
        // Current limit (IMPORTANT for NEO 550)
        leftConfig.smartCurrentLimit(20);
        rightConfig.smartCurrentLimit(20);
        // Apply configs to motors
        intakeLeft.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        intakeRight.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    // ==============================
    // Intake Control
    // ==============================

    public void runIntake(double speed) {
        intakeLeft.set(speed);
        intakeRight.set(speed);
    }

    public void intakeIn() {
        runIntake(0.4);
    }

    public void intakeOut() {
        runIntake(-0.4);
    }

    public void stopIntake() {
        runIntake(0.0);
    }

    // ==============================
    // Pivot Control
    // ==============================

    public void pivotUp() {
        pivotSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void pivotDown() {
        pivotSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void stopPivot() {
        pivotSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void togglePivot() {
        pivotSolenoid.toggle();
    }

    @Override
    public void periodic() {
        // Optional: Add SmartDashboard telemetry here
        // Motor Output (-1 to 1)
        SmartDashboard.putNumber("Intake Left Output", intakeLeft.get());
        SmartDashboard.putNumber("Intake Right Output", intakeRight.get());

        // Motor Current (amps)
        SmartDashboard.putNumber("Intake Left Current", intakeLeft.getOutputCurrent());
        SmartDashboard.putNumber("Intake Right Current", intakeRight.getOutputCurrent());

        // Motor Velocity (RPM)
        SmartDashboard.putNumber(
            "Intake Left Velocity",
            intakeLeft.getEncoder().getVelocity()
        );

        SmartDashboard.putNumber(
            "Intake Right Velocity",
            intakeRight.getEncoder().getVelocity()
        );

        // Pivot State
        SmartDashboard.putString(
            "Intake Pivot State",
            pivotSolenoid.get().toString()
        );

        // Intake Running Boolean
        SmartDashboard.putBoolean(
            "Intake Running",
            Math.abs(intakeLeft.get()) > 0.05
        );
    }
}