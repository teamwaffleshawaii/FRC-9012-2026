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

    private final SparkFlex intakeVortex = new SparkFlex(19, MotorType.kBrushless);
    
    // ==============================
    // Pneumatics (REV PH CAN ID: 2)
    // ==============================

    private final DoubleSolenoid pivotSolenoid =
        new DoubleSolenoid(
            2, // REV Pneumatic Hub CAN ID
            PneumaticsModuleType.REVPH,
            1, // forward channel
            0  // reverse channel
        );

    // ==============================
    // Constructor
    // ==============================

    public IntakeSubsystem() {
    //     SparkMaxConfig leftConfig = new SparkMaxConfig();
    //     SparkMaxConfig rightConfig = new SparkMaxConfig();
    //     // Inversion
    //     rightConfig.inverted(true);
    //     // Brake mode
    //     leftConfig.idleMode(IdleMode.kBrake);
    //     rightConfig.idleMode(IdleMode.kBrake);
    //     // Current limit (IMPORTANT for NEO 550)
    //     leftConfig.smartCurrentLimit(20);
    //     rightConfig.smartCurrentLimit(20);
    //     // Apply configs to motors
    //     intakeLeft.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    //     intakeRight.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
     }

    // ==============================
    // Intake Control
    // ==============================

    public void runIntake(double speed) {
        intakeVortex.set(speed);
    }

    public void intakeIn() {
        runIntake(1);
    }

    public void intakeInFullPower() {
        runIntake(1.0);
    }

    public void intakeOut() {
        runIntake(-0.35);
    }

    //For during transfer to prevent FUELs from being stuck by the intake
    public void intakeTransfer() {
        runIntake(0.85);
    }

    public void intakeStop() {
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
     
    }
}