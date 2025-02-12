// File: LocalizationSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

public class LocalizationSubsystem extends SubsystemBase {
    private final DriveSubsystem driveSubsystem;
    private final SwerveDrivePoseEstimator poseEstimator;
    private Pose2d capturedPose = null;  // Stores the locked-in pose for trajectory planning

    public LocalizationSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(driveSubsystem.getHeading()),
            driveSubsystem.getModulePositions(),
            new Pose2d(), // initial pose; adjust if needed
            VecBuilder.fill(0.1, 0.1, 0.1),   // state standard deviations [meters, meters, radians]
            VecBuilder.fill(0.5, 0.5, 0.5)    // vision measurement standard deviations [meters, meters, radians]
        );
    }

    @Override
    public void periodic() {
        // Update odometry using the drive subsystem's sensor values.
        poseEstimator.update(
            Rotation2d.fromDegrees(driveSubsystem.getHeading()),
            driveSubsystem.getModulePositions()
        );

        // NOTE: Since MegaTag2 is not available with your current Limelight version,
        // we are not fusing vision measurements here. In the future, if you have an
        // alternative vision pipeline that provides a relative pose to an AprilTag,
        // you could add that measurement here.

        // Publish the estimated pose for debugging purposes.
        Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("Estimated X", estimatedPose.getTranslation().getX());
        SmartDashboard.putNumber("Estimated Y", estimatedPose.getTranslation().getY());
        SmartDashboard.putNumber("Estimated Rotation", estimatedPose.getRotation().getDegrees());
    }

    /**
     * Call this method to capture the current pose as the starting pose for trajectory planning.
     *
     * @param currentPose The pose to capture.
     */
    public void captureStartPose(Pose2d currentPose) {
        this.capturedPose = currentPose;
        SmartDashboard.putString("Captured Pose", currentPose.toString());
    }

    public Pose2d getCapturedPose() {
        return capturedPose;
    }
}
