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
import frc.robot.LimelightHelpers; // Ensure this is the correct import based on your setup

public class LocalizationSubsystem extends SubsystemBase {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    
    // Pose estimator that fuses odometry and vision.
    private final SwerveDrivePoseEstimator poseEstimator;

    public LocalizationSubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        
        // Initialize the pose estimator using:
        // - Your robot's kinematics.
        // - The current gyro reading (converted to a Rotation2d).
        // - The current module positions from your drive subsystem.
        // - An initial pose (here we assume (0,0,0), but you can set this as needed).
        // - Standard deviations for the state and vision measurement.
        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(driveSubsystem.getHeading()),
            driveSubsystem.getModulePositions(),
            new Pose2d(), // initial pose; update as necessary
            VecBuilder.fill(0.1, 0.1, 0.1), // State standard deviations [meters, meters, radians]
            VecBuilder.fill(0.5, 0.5, 0.5)  // Vision measurement standard deviations [meters, meters, radians]
        );
    }

    @Override
    public void periodic() {
        // Update the pose estimator with the current odometry data.
        // This uses the current gyro heading and module positions.
        poseEstimator.update(
            Rotation2d.fromDegrees(driveSubsystem.getHeading()),
            driveSubsystem.getModulePositions()
        );
        
        // Retrieve vision-based pose estimate from MegaTag2 via LimelightHelpers.
        LimelightHelpers.PoseEstimate visionEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        
        // Use the vision measurement if at least one tag is detected.
        if (visionEstimate.tagCount > 0) {
            // Optionally: If the robot is turning very fast, you might want to ignore the vision update.
            // For example:
            // if (Math.abs(driveSubsystem.getTurnRate()) < someThreshold) { ... }
            
            // Feed the vision measurement into the estimator.
            poseEstimator.addVisionMeasurement(visionEstimate.pose, visionEstimate.timestampSeconds);
        }
        
        // Optionally publish the estimated pose to SmartDashboard for debugging.
        Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("Estimated X", estimatedPose.getTranslation().getX());
        SmartDashboard.putNumber("Estimated Y", estimatedPose.getTranslation().getY());
        SmartDashboard.putNumber("Estimated Rotation", estimatedPose.getRotation().getDegrees());
    }
    
    /**
     * Returns the current estimated pose of the robot.
     * This pose fuses both odometry and vision data.
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
