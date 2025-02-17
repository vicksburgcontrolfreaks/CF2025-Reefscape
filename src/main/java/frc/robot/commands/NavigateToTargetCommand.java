// File: NavigateToTargetCommand.java
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

public class NavigateToTargetCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final Pose2d m_targetPose;
    private SwerveControllerCommand m_trajectoryCommand;
    private Pose2d m_startPose;

    /**
     * Constructs a NavigateToTargetCommand.
     * @param driveSubsystem The drive subsystem.
     * @param targetPose The desired target pose.
     */
    public NavigateToTargetCommand(DriveSubsystem driveSubsystem, Pose2d targetPose) {
        m_driveSubsystem = driveSubsystem;
        m_targetPose = targetPose;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        // Get the current robot pose (fused with vision, if available).
        m_startPose = m_driveSubsystem.getPose();
        
        // Create a trajectory configuration.
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.DriveConstants.kDriveKinematics);
        
        // Generate the trajectory from the current pose to the target pose.
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                m_startPose,
                List.of(),  // No intermediate waypoints; you can add if needed.
                m_targetPose,
                config
        );
        
        // Create a Profiled PID controller for rotational control.
        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Create the SwerveControllerCommand to follow the trajectory.
        m_trajectoryCommand = new SwerveControllerCommand(
                trajectory,
                m_driveSubsystem::getPose,
                Constants.DriveConstants.kDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                m_driveSubsystem::setModuleStates,
                m_driveSubsystem
        );
        
        // Reset odometry to the starting pose for accurate trajectory following.
        m_driveSubsystem.resetOdometry(m_startPose);
        m_trajectoryCommand.initialize();
    }

    @Override
    public void execute() {
        m_trajectoryCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return m_trajectoryCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_trajectoryCommand.end(interrupted);
        m_driveSubsystem.drive(0, 0, 0, false);
    }
}
