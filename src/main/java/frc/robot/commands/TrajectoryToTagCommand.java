// File: TrajectoryToTagCommand.java
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.List;

public class TrajectoryToTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private SwerveControllerCommand internalCommand;
    private boolean validMeasurement = true;
    private final boolean isLeft;  // True for left offset, false for right offset

    /**
     * Constructor for TrajectoryToTagCommand.
     * @param driveSubsystem The drive subsystem.
     * @param visionSubsystem The vision subsystem.
     * @param isLeft If true, the destination is offset to the left; if false, to the right.
     */
    public TrajectoryToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, boolean isLeft) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.isLeft = isLeft;
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        // 1. Get the current robot pose from odometry.
        Pose2d currentPose = driveSubsystem.getPose();
        
        // 2. Retrieve the AprilTag's relative pose from the vision subsystem.
        Pose2d tagRelativePose = visionSubsystem.getAprilTagPoseRelative();
        if (tagRelativePose == null) {
            validMeasurement = false;
            System.out.println("No AprilTag detected. Command will not move the robot.");
            return;
        }
        validMeasurement = true;
        
        // 3. Convert the relative pose into a Transform2d.
        Transform2d relativeTransform = new Transform2d(new Pose2d(), tagRelativePose);
        
        // 4. Compute the tag's global pose by applying the relative transform to the current pose.
        Pose2d tagGlobalPose = currentPose.transformBy(relativeTransform);
        
        // 5. Define the desired offset:
        //    1 inch forward always, and 7 inches to the left if isLeft is true, or 7 inches to the right if false.
        double offsetX = 15 * 0.0254;   // 1 inch in meters (≈ 0.0254 m)
        double offsetY = 7 * 0.0254;   // 7 inches in meters (≈ 0.1778 m)
        // Adjust lateral offset based on isLeft. Positive offsetY means to the left.
        if (!isLeft) {
            offsetY = -offsetY;
        }
        Pose2d offsetPose = new Pose2d(offsetX, offsetY, new Rotation2d(0));
        
        // 6. Compute the destination pose by applying the offset relative to the tag's global pose.
        Pose2d destinationPose = tagGlobalPose.transformBy(new Transform2d(new Pose2d(), offsetPose));
        
        // 7. Create a trajectory configuration.
        TrajectoryConfig config = new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.DriveConstants.kDriveKinematics);
        
        // 8. Generate a trajectory from the current pose to the destination pose.
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            currentPose,
            List.of(),  // Optionally, add intermediate waypoints if needed.
            destinationPose,
            config
        );
        
        // 9. Create a theta controller for rotational control.
        var thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        // 10. Create the SwerveControllerCommand to follow the trajectory.
        internalCommand = new SwerveControllerCommand(
            trajectory,
            driveSubsystem::getPose,                    // Supplier for current pose.
            Constants.DriveConstants.kDriveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            driveSubsystem::setModuleStates,            // Function to set module states.
            driveSubsystem
        );
        
        // 11. Reset odometry to the starting pose for accurate tracking.
        driveSubsystem.resetOdometry(currentPose);
        
        // 12. Initialize the internal trajectory-following command.
        internalCommand.initialize();
    }
    
    @Override
    public void execute() {
        if (validMeasurement && internalCommand != null) {
            internalCommand.execute();
        }
    }
    
    @Override
    public boolean isFinished() {
        if (!validMeasurement) {
            return true;
        }
        return internalCommand.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        if (validMeasurement && internalCommand != null) {
            internalCommand.end(interrupted);
        }
        driveSubsystem.drive(0, 0, 0, false);
    }
    
    @Override
    public String getName() {
        return "TrajectoryToTagCommand";
    }
}
