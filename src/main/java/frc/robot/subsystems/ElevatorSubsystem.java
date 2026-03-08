package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax m_elevatorR = new SparkMax(11, MotorType.kBrushless);
    private final SparkMax m_elevatorL = new SparkMax(12, MotorType.kBrushless);
    
    private final SparkClosedLoopController leaderController;
    private final SparkClosedLoopController followerController;
    
    private final SparkMaxConfig baseConfig = new SparkMaxConfig();

    // Finalized PID Constants
    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0001;

    private static final double kTopPosition = 39.0;
    private static final double kBottomPosition = 0.0;
    private static final double kMaxPosition = 39.0;
    private static final double kMinPosition = 0.0;

    private double targetPosition = 0.0;

    public ElevatorSubsystem() {
        // Define shared settings
        baseConfig.idleMode(IdleMode.kBrake);
        baseConfig.smartCurrentLimit(40);
        baseConfig.closedLoop.p(kP);
        baseConfig.closedLoop.i(kI);
        baseConfig.closedLoop.d(kD);
        baseConfig.closedLoop.outputRange(-0.6, 0.6);

        // Configure Leader
        m_elevatorR.configure(baseConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Configure Follower
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.apply(baseConfig); 
        followerConfig.inverted(true); 

        m_elevatorL.configure(followerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        leaderController = m_elevatorR.getClosedLoopController();
        followerController = m_elevatorL.getClosedLoopController();

        // Zero both encoders
        m_elevatorR.getEncoder().setPosition(0);
        m_elevatorL.getEncoder().setPosition(0);

        // --- DEPRECATED TUNING TOOLS ---
        // SmartDashboard.putNumber("Elevator P", kP);
        // SmartDashboard.putNumber("Elevator I", kI);
        // SmartDashboard.putNumber("Elevator D", kD);
    }

    public void goTop() { setPosition(kTopPosition); }
    public void goBottom() { setPosition(kBottomPosition); }

    private void setPosition(double position) {
        targetPosition = position;
        leaderController.setReference(position, SparkMax.ControlType.kPosition);
        followerController.setReference(position, SparkMax.ControlType.kPosition);
    }

    public double getPosition() {
        return (m_elevatorR.getEncoder().getPosition() + m_elevatorL.getEncoder().getPosition()) / 2.0;
    }

    public boolean isAtTarget() {
        return Math.abs(getPosition() - targetPosition) < 0.5;
    }

    public void adjustPosition(double delta) {
        double newTarget = Math.max(kMinPosition, Math.min(kMaxPosition, targetPosition + delta));
        setPosition(newTarget);
    }

    /* // REMOVED FOR EFFICIENCY: No longer need to check for PID updates during match
    private void updatePIDFromDashboard() {
        double p = SmartDashboard.getNumber("Elevator P", kP);
        ...
    }
    */

    @Override
    public void periodic() {
        // updatePIDFromDashboard(); // Removed to save CPU cycles

        // Minimal Telemetry: Keeping only what the driver needs to know
        SmartDashboard.putNumber("Elevator/Pos", getPosition());
        SmartDashboard.putBoolean("Elevator/At Target", isAtTarget());

        // --- REMOVED NON-ESSENTIAL TELEMETRY ---
        // SmartDashboard.putNumber("Elevator/Setpoint", targetPosition);
        // SmartDashboard.putNumber("Left Elevator", m_elevatorL.getEncoder().getPosition());
        // SmartDashboard.putNumber("Right Elevator", m_elevatorR.getEncoder().getPosition());
    }
}
/* FOR TUNING OF ELEVATOR
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax m_elevatorR = new SparkMax(11, MotorType.kBrushless);
    private final SparkMax m_elevatorL = new SparkMax(12, MotorType.kBrushless);
    
    // Now both motors have their own controllers
    private final SparkClosedLoopController leaderController;
    private final SparkClosedLoopController followerController;
    
    private final SparkMaxConfig baseConfig = new SparkMaxConfig();

    private double lastP = 1.0;
    private double lastI = 0.0;
    private double lastD = 0.0001;

    private static final double kTopPosition = 39.0;
    private static final double kBottomPosition = 0.0;
    private static final double kMaxPosition = 39.0;
    private static final double kMinPosition = 0.0;

    private double targetPosition = 0.0;

    public ElevatorSubsystem() {
        // Define shared settings
        baseConfig.idleMode(IdleMode.kBrake);
        baseConfig.smartCurrentLimit(40);
        baseConfig.closedLoop.p(lastP);
        baseConfig.closedLoop.i(lastI);
        baseConfig.closedLoop.d(lastD);
        baseConfig.closedLoop.outputRange(-0.6, 0.6);

        // Configure Leader
        m_elevatorR.configure(baseConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Configure Follower with the SAME PID config (but inverted if needed)
        // Note: .inverted(true) replaces .follow(m_elevatorR, true) for direction
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.apply(baseConfig); 
        followerConfig.inverted(true); // Manually handle the inversion here

        m_elevatorL.configure(followerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        leaderController = m_elevatorR.getClosedLoopController();
        followerController = m_elevatorL.getClosedLoopController();

        // Zero both encoders
        m_elevatorR.getEncoder().setPosition(0);
        m_elevatorL.getEncoder().setPosition(0);

        SmartDashboard.putNumber("Elevator P", lastP);
        SmartDashboard.putNumber("Elevator I", lastI);
        SmartDashboard.putNumber("Elevator D", lastD);
    }

    public void goTop() { setPosition(kTopPosition); }
    public void goBottom() { setPosition(kBottomPosition); }

    private void setPosition(double position) {
        targetPosition = position;
        // Command BOTH motors to the same target
        leaderController.setReference(position, SparkMax.ControlType.kPosition);
        followerController.setReference(position, SparkMax.ControlType.kPosition);
    }

    public double getPosition() {
        // Average the two for the "system" position, or just use m_elevatorR
        return (m_elevatorR.getEncoder().getPosition() + m_elevatorL.getEncoder().getPosition()) / 2.0;
    }

    public boolean isAtTarget() {
        return Math.abs(getPosition() - targetPosition) < 0.5;
    }

    public void adjustPosition(double delta) {
        double newTarget = Math.max(kMinPosition, Math.min(kMaxPosition, targetPosition + delta));
        setPosition(newTarget);
    }

    private void updatePIDFromDashboard() {
        double p = SmartDashboard.getNumber("Elevator P", lastP);
        double i = SmartDashboard.getNumber("Elevator I", lastI);
        double d = SmartDashboard.getNumber("Elevator D", lastD);

        if (p != lastP || i != lastI || d != lastD) {
            baseConfig.closedLoop.p(p).i(i).d(d);
            
            // Re-apply to both motors
            m_elevatorR.configure(baseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            
            // Re-apply to m_elevatorL (keeping inversion)
            SparkMaxConfig fUpdate = new SparkMaxConfig();
            fUpdate.apply(baseConfig).inverted(true);
            m_elevatorL.configure(fUpdate, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            lastP = p; lastI = i; lastD = d;
        }
    }

    @Override
    public void periodic() {
        updatePIDFromDashboard();

        // Updated Telemetry - getting fresh values from encoders
        SmartDashboard.putNumber("Elevator/Measured", getPosition());
        SmartDashboard.putNumber("Elevator/Setpoint", targetPosition);
        SmartDashboard.putNumber("Left Elevator", m_elevatorL.getEncoder().getPosition());
        SmartDashboard.putNumber("Right Elevator", m_elevatorR.getEncoder().getPosition());
        SmartDashboard.putBoolean("Elevator At Target", isAtTarget());
    }
}*/


/*
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax m_elevatorR = new SparkMax(11, MotorType.kBrushless);
    private final SparkMax m_elevatorL = new SparkMax(12, MotorType.kBrushless);

    private final SparkClosedLoopController controller;

    private static final double kTopPosition = 39.0; // can theoretically go to 40
    private static final double kBottomPosition = 0.0;

    private static final double kMaxPosition = 39.0;
    private static final double kMinPosition = 0.0;
    private static final double kManualStep = 1.0;

    private double targetPosition = 0.0;

    public ElevatorSubsystem() {

        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig.idleMode(IdleMode.kBrake);
        baseConfig.smartCurrentLimit(40);

        ClosedLoopConfig pidConfig = new ClosedLoopConfig();
        pidConfig.p(1.0);
        pidConfig.i(0.0);
        pidConfig.d(0.0001);
        pidConfig.outputRange(-0.6, 0.6);

        baseConfig.apply(pidConfig);

        // Leader
        m_elevatorR.configure(baseConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Follower (inverted)
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kBrake);
        followerConfig.smartCurrentLimit(40);
        followerConfig.follow(m_elevatorR, true);

        m_elevatorL.configure(followerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        controller = m_elevatorR.getClosedLoopController();

        // Zero at startup
        m_elevatorR.getEncoder().setPosition(0);
    }

    // ---------------- Elevator Commands ----------------

    public void goTop() {
        setPosition(kTopPosition);
    }

    public void goBottom() {
        setPosition(kBottomPosition);
    }

    private void setPosition(double position) {
    targetPosition = position;
    controller.setReference(position, SparkMax.ControlType.kPosition);
}

    public boolean isAtTarget() {
        return Math.abs(getPosition() - targetPosition) < 0.5;
    }

    public double getPosition() {
        return m_elevatorR.getEncoder().getPosition();
    }

    public void adjustPosition(double delta) {

    double newTarget = targetPosition + delta;

    // Clamp between limits
    newTarget = Math.max(kMinPosition, Math.min(kMaxPosition, newTarget));

    setPosition(newTarget);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putNumber("Elevator Target", targetPosition);
        SmartDashboard.putBoolean("Elevator At Target", isAtTarget());
    }
}*/
