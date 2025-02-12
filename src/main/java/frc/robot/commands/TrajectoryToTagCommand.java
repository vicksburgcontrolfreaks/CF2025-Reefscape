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
    private boolean validMeasurement = true; // Flag to indicate if an AprilTag was detected

    public TrajectoryToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // 1. Get the current robot pose from odometry.
        Pose2d currentPose = driveSubsystem.getPose();

        // 2. Retrieve the AprilTag's relative pose from the vision subsystem.
        //    This method should return a Pose2d representing the tag's pose relative to the robot,
        //    or null if no tag is detected.
        Pose2d tagRelativePose = visionSubsystem.getAprilTagPoseRelative();

        // If no tag is detected, mark the measurement as invalid and exit.
        if (tagRelativePose == null) {
            validMeasurement = false;
            System.out.println("No AprilTag detected. Command will not move the robot.");
            return;
        }
        validMeasurement = true;

        // 3. Convert the relative pose into a Transform2d.
        Transform2d relativeTransform = new Transform2d(new Pose2d(), tagRelativePose);

        // 4. Calculate the tag's global pose using the current odometry.
        Pose2d targetTagPose = currentPose.transformBy(relativeTransform);

        // 5. Define the desired offset relative to the tag:
        //    15 inches forward and 7 inches to the left.
        //    Convert inches to meters: 15 in ≈ 0.381 m, 7 in ≈ 0.1778 m.
        double offsetX = 15 * 0.0254; // ~0.381 m
        double offsetY = 7 * 0.0254;  // ~0.1778 m
        Pose2d offsetPose = new Pose2d(offsetX, offsetY, new Rotation2d(0));

        // 6. Calculate the destination pose by applying the offset relative to the tag's global pose.
        Pose2d destinationPose = targetTagPose.transformBy(new Transform2d(new Pose2d(), offsetPose));

        // 7. Create a trajectory configuration.
        TrajectoryConfig config = new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.DriveConstants.kDriveKinematics);

        // 8. Generate a trajectory from the current pose to the destination pose.
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            currentPose,
            List.of(),  // No intermediate waypoints.
            destinationPose,
            config
        );

        // 9. Create a theta controller for rotational control.
        var thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 10. Create the internal SwerveControllerCommand to follow the trajectory.
        internalCommand = new SwerveControllerCommand(
            trajectory,
            driveSubsystem::getPose,                    // Supplier for current pose.
            Constants.DriveConstants.kDriveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            driveSubsystem::setModuleStates,            // Function to set the module states.
            driveSubsystem
        );

        // 11. Reset odometry to the starting pose.
        driveSubsystem.resetOdometry(currentPose);

        // 12. Initialize the internal command.
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
        // If no valid measurement was obtained, finish immediately.
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
        // Ensure the drivetrain is stopped.
        driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public String getName() {
        return "TrajectoryToTagCommand";
    }
}
