// File: NavigateToProcessorCommand.java
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

public class NavigateToProcessorCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private SwerveControllerCommand m_trajectoryCommand;

    public NavigateToProcessorCommand(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        // 1. Get the current (fused) robot pose.
        Pose2d currentPose = m_driveSubsystem.getPose();

        // 2. Determine alliance and select target pose.
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Pose2d targetPose;
        if (alliance == DriverStation.Alliance.Blue) {
            // Blue alliance: x=11.529, y=7.399, theta=-180° (-π radians)
            targetPose = new Pose2d(11.529, 7.399, new Rotation2d(-Math.PI));
        } else {  // Red alliance
            // Red alliance: x=6.099, y=0.595, theta=0° (0 radians)
            targetPose = new Pose2d(6.099, 0.595, new Rotation2d(0));
        }

        // 3. Create trajectory configuration.
        TrajectoryConfig config = new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.DriveConstants.kDriveKinematics);

        // 4. Generate trajectory from current pose to target pose.
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            currentPose,
            List.of(), // You can add intermediate waypoints if desired.
            targetPose,
            config
        );

        // 5. Create a theta controller for rotational control.
        var thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 6. Create the SwerveControllerCommand to follow the trajectory.
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

        // 7. Reset odometry to the starting pose.
        m_driveSubsystem.resetOdometry(currentPose);
        m_trajectoryCommand.initialize();
    }

    @Override
    public void execute() {
        m_trajectoryCommand.execute();
    }

    @Override
    public boolean isFinished() {
        // If a collision is detected, abort the command.
        if (m_driveSubsystem.isCollisionDetected()) {
            System.out.println("Collision detected, aborting NavigateToProcessorCommand.");
            return true;
        }
        return m_trajectoryCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_trajectoryCommand.end(interrupted);
        m_driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public String getName() {
        return "NavigateToProcessorCommand";
    }
}
