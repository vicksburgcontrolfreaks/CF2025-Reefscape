
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
import edu.wpi.first.wpilibj.DriverStation;

public class LocalizationSubsystem extends SubsystemBase {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d m_field = new Field2d();
    private Pose2d capturedPose = null; // For trajectory planning

    public LocalizationSubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        
        // Set an initial pose based on alliance.
        // For red alliance: x=10, y=4, 0° heading.
        // For blue alliance: x=7.3, y=4, -180° heading.
        Pose2d initialPose;
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            initialPose = new Pose2d(10, 4, new Rotation2d(0));  // 0° in radians
        } else {
            initialPose = new Pose2d(7.3, 4, new Rotation2d(Math.toRadians(180)));  // -180° in radians (-π)
        }
        
        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(driveSubsystem.getHeading()),
            driveSubsystem.getModulePositions(),
            initialPose,  // Use our known starting pose
            VecBuilder.fill(0.1, 0.1, 0.1), // State standard deviations: [meters, meters, radians]
            VecBuilder.fill(0.05, 0.05, 0.05)  // Vision measurement standard deviations.
        );
        
        // Publish the Field2d widget to SmartDashboard.
        // SmartDashboard.putData("Field", m_field);
    }

    @Override
public void periodic() {
    // Set Limelight IMU mode and update robot orientation.
    LimelightHelpers.SetIMUMode("limelight", 3);
    LimelightHelpers.SetRobotOrientation("limelight", driveSubsystem.getHeading(), 0, 0, 0, 0, 0);

    // Update odometry using sensor values from the drive subsystem.
    poseEstimator.update(
        Rotation2d.fromDegrees(driveSubsystem.getHeading()),
        driveSubsystem.getModulePositions()
    );

    // Retrieve vision estimate.
    LimelightHelpers.PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    
    // Define thresholds (you may need to tune these)
    double maxRotationalRate = 720; // degrees per second; adjust as needed
    double maxTranslationalSpeed = 1.0; // m/s; adjust as needed
    
    // Check current speeds (assuming driveSubsystem provides a method to get chassis speeds)
    double currentRotRate = Math.abs(driveSubsystem.getTurnRate());
    // For translation, you might compute a combined speed from your module velocities or a method from your drive subsystem.
    double currentTransSpeed = driveSubsystem.getChassisSpeed();  // Example method; you may have to implement one.
    
    // Only update vision measurement if the robot is moving slowly.
    if (mt2Estimate != null && mt2Estimate.tagCount > 0 && 
        currentRotRate < maxRotationalRate && currentTransSpeed < maxTranslationalSpeed) {
        // Trust vision data heavily when moving slowly.
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.01, 0.01, 0.01));
        poseEstimator.addVisionMeasurement(mt2Estimate.pose, mt2Estimate.timestampSeconds);
    } else {
        // Optionally, if moving fast, you can either not update or set very high uncertainties.
        // For example, you could set:
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(10, 10, 10));
    }

    // Publish the estimated pose.
    Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
    m_field.setRobotPose(estimatedPose);
    // SmartDashboard.putNumber("Estimated X", estimatedPose.getTranslation().getX());
    // SmartDashboard.putNumber("Estimated Y", estimatedPose.getTranslation().getY());
    // SmartDashboard.putNumber("Estimated Rotation", estimatedPose.getRotation().getDegrees());
    SmartDashboard.putString("Estimated Pose", estimatedPose.toString());
}


    /**
     * Captures the current pose to be used later for trajectory planning.
     * @param currentPose The pose to capture.
     */
    public void captureStartPose(Pose2d currentPose) {
        this.capturedPose = currentPose;
        // SmartDashboard.putString("Captured Pose", currentPose.toString());
    }

    public Pose2d getCapturedPose() {
        return capturedPose;
    }

    /**
     * Returns the current estimated robot pose.
     * @return The estimated pose.
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Field2d getField() {
        return m_field;
    }
}
