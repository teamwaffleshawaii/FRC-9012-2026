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

    private final SparkMax leader = new SparkMax(11, MotorType.kBrushless);
    private final SparkMax follower = new SparkMax(12, MotorType.kBrushless);

    private final SparkClosedLoopController controller;

    private static final double kTopPosition = 35.5;
    private static final double kBottomPosition = 0.0;

    private static final double kMaxPosition = 35.5;
    private static final double kMinPosition = 0.0;
    private static final double kManualStep = 1.0;

    private double targetPosition = 0.0;

    public ElevatorSubsystem() {

        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig.idleMode(IdleMode.kBrake);
        baseConfig.smartCurrentLimit(40);

        ClosedLoopConfig pidConfig = new ClosedLoopConfig();
        pidConfig.p(0.1);
        pidConfig.i(0.0);
        pidConfig.d(0.0001);
        pidConfig.outputRange(-0.6, 0.6);

        baseConfig.apply(pidConfig);

        // Leader
        leader.configure(baseConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Follower (inverted)
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kBrake);
        followerConfig.smartCurrentLimit(40);
        followerConfig.follow(leader, true);

        follower.configure(followerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        controller = leader.getClosedLoopController();

        // Zero at startup
        leader.getEncoder().setPosition(0);
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
        return leader.getEncoder().getPosition();
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
}
