// File: DriveToPoseCommand.java
package frc.robot.commands;

import java.util.List;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.LimelightHelpers;

public class DriveToPoseCommand extends Command {
    private SwerveControllerCommand internalCommand;
    private final DriveSubsystem driveSubsystem;
    private final Pose2d targetPose;

    public DriveToPoseCommand(DriveSubsystem driveSubsystem, Pose2d targetPose) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Get the current odometry as the default starting pose.
        Pose2d startPose = driveSubsystem.getPose();
        // Check if a vision estimate is available (e.g., via Limelight with MegaTag2).
        var visionEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (visionEstimate != null && visionEstimate.tagCount > 0) {
            // Use the vision pose if a tag is visible.
            startPose = visionEstimate.pose;
        }

        // Build a trajectory from the starting pose to the target pose.
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                startPose,
                List.of(),  // No interior waypoints.
                targetPose,
                config
        );

        // Create PID controllers for translation.
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        // Create a motion-profiled PID controller for rotation.
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Create the SwerveControllerCommand.
        internalCommand = new SwerveControllerCommand(
                trajectory,
                driveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                driveSubsystem::setModuleStates,
                driveSubsystem
        );

        // Reset odometry to the vision-based (or default) starting pose.
        driveSubsystem.resetOdometry(startPose);
        internalCommand.initialize();
    }

    @Override
    public void execute() {
        if (internalCommand != null) {
            internalCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return internalCommand != null && internalCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (internalCommand != null) {
            internalCommand.end(interrupted);
        }
        // Stop the drive subsystem.
        driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public Set<edu.wpi.first.wpilibj2.command.Subsystem> getRequirements() {
        return internalCommand != null ? internalCommand.getRequirements() : Set.of();
    }
}
