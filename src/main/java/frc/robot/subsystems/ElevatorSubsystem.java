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


    @Override
    public void periodic() {
       
    }
}