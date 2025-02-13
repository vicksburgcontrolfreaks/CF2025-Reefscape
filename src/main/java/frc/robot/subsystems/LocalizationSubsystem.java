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
import frc.robot.LimelightHelpers;

public class LocalizationSubsystem extends SubsystemBase {
    private final DriveSubsystem driveSubsystem;
    private final SwerveDrivePoseEstimator poseEstimator;
    private Pose2d capturedPose = null; // Stores the locked-in pose for trajectory planning

    public LocalizationSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                Rotation2d.fromDegrees(driveSubsystem.getHeading()),
                driveSubsystem.getModulePositions(),
                new Pose2d(), // initial pose; adjust if needed
                VecBuilder.fill(0.1, 0.1, 0.1), // state standard deviations [meters, meters, radians]
                VecBuilder.fill(0.5, 0.5, 0.5) // vision measurement standard deviations [meters, meters, radians]
        );
    }

    @Override
    public void periodic() {
        // Set the Limelight to use its integrated IMU for MegaTag2 localization.
        LimelightHelpers.SetIMUMode("limelight", 2);

        // Update odometry using the drive subsystem's sensor values.
        poseEstimator.update(
                Rotation2d.fromDegrees(driveSubsystem.getHeading()),
                driveSubsystem.getModulePositions());

        // Retrieve the MegaTag2 vision estimate.
        LimelightHelpers.PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (mt2Estimate.tagCount > 0) {
            poseEstimator.addVisionMeasurement(mt2Estimate.pose, mt2Estimate.timestampSeconds);
        }

        // Publish the estimated pose for debugging.
        Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("Estimated X", estimatedPose.getTranslation().getX());
        SmartDashboard.putNumber("Estimated Y", estimatedPose.getTranslation().getY());
        SmartDashboard.putNumber("Estimated Rotation", estimatedPose.getRotation().getDegrees());
    }
}