// File: LocalizationSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

public class LocalizationSubsystem extends SubsystemBase {
    private final DriveSubsystem driveSubsystem;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d m_field = new Field2d();

    private Pose2d capturedPose = null; // Stores the locked-in pose for trajectory planning

    public LocalizationSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                Rotation2d.fromDegrees(driveSubsystem.getHeading()),
                driveSubsystem.getModulePositions(),
                new Pose2d(), // initial pose; adjust if needed
                VecBuilder.fill(0.1, 0.1, 0.1), // state standard deviations [meters, meters, radians]
                VecBuilder.fill(0.5, 0.5, 0.5) // default vision measurement standard deviations
        );
        // Publish the Field2d widget to SmartDashboard.
        SmartDashboard.putData("Field", m_field);
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
            // If more than one tag is seen, trust vision more by reducing standard
            // deviations.
            double visionStdDev = (mt2Estimate.tagCount > 1) ? 0.3 : 0.5;
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionStdDev, visionStdDev, visionStdDev));
            poseEstimator.addVisionMeasurement(mt2Estimate.pose, mt2Estimate.timestampSeconds);
        }
      

        // Publish the estimated pose for debugging.
        Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
          
        // Update the Field2d with the current robot pose.
          m_field.setRobotPose(estimatedPose);
          
        SmartDashboard.putNumber("Estimated X", estimatedPose.getTranslation().getX());
        SmartDashboard.putNumber("Estimated Y", estimatedPose.getTranslation().getY());
        SmartDashboard.putNumber("Estimated Rotation", estimatedPose.getRotation().getDegrees());
    }

    /**
     * Captures the current pose to be used later for trajectory planning.
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

    /**
     * Returns the current estimated pose of the robot.
     *
     * @return The fused pose estimate.
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
