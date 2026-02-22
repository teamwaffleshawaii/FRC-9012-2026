
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.LimelightHelpers;

public class AlignToAprilTagCommand extends Command {

    private final DriveSubsystem drive;
    private final int targetAprilTagID;
    private final double timeoutSeconds;

    private final Timer timer = new Timer();

    // ===== Alignment Constants =====
    private static final double targetDistanceMetersTZ = 1.40;
    private static final double targetDistanceMetersTX = 0.45;
    private static final double targetHeadingRY = 0;

    private static final double distanceKp = 0.5;
    private static final double strafeKp = 0.5;
    private static final double steeringKp = 0.5;


    public AlignToAprilTagCommand(
            DriveSubsystem drive,
            int targetAprilTagID,
            double timeoutSeconds
    ) {
        this.drive = drive;
        this.targetAprilTagID = targetAprilTagID;
        this.timeoutSeconds = timeoutSeconds;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {

        if (!LimelightHelpers.getTV("limelight")) {
            drive.drive(0, 0, 0, false);
            return;
        }

        // botPose: [x, y, z, roll, pitch, yaw]
        double[] botPose = LimelightHelpers.getBotPose_TargetSpace("limelight");

        double currentStrafeX   = botPose[0];
        double currentDistanceZ = botPose[2];
        double currentYaw       = botPose[4];
        
        SmartDashboard.putNumber("TX: ", currentStrafeX);
        SmartDashboard.putNumber("TZ: ", currentDistanceZ);
        SmartDashboard.putNumber("Yaw: ", currentYaw);

        double distanceError = -targetDistanceMetersTZ - currentDistanceZ;
        double strafeError   = -targetDistanceMetersTX - currentStrafeX;
        double steeringError = targetHeadingRY - currentYaw;
        double radError = Math.toRadians(steeringError);

        // Deadbands
        if (Math.abs(distanceError) < 0.02) distanceError = 0;
        if (Math.abs(strafeError) < 0.02) strafeError = 0;
        if (Math.abs(steeringError) < 2.0) steeringError = 0;

        drive.drive(
            distanceError * distanceKp,
            -strafeError * strafeKp,
            -radError * steeringKp,
            false
        );
    }

    @Override
    public boolean isFinished() {
        boolean timedOut = timer.hasElapsed(timeoutSeconds);

        // End early if fully aligned
        if (LimelightHelpers.getTV("limelight")) {
            double[] pose = LimelightHelpers.getBotPose_TargetSpace("limelight");
            boolean aligned =
                Math.abs(pose[2] + targetDistanceMetersTZ) < 0.03 &&
                Math.abs(pose[0] + targetDistanceMetersTX) < 0.03 &&
                Math.abs(pose[4]) < 3.0;

            return aligned || timedOut;
        }

        return timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, false);
    }
}



