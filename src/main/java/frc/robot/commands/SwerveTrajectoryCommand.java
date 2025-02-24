// File: SwerveTrajectoryCommand.java
package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveTrajectoryCommand extends Command {
    private final SwerveControllerCommand swerveCommand;

    /**
     * Constructs a SwerveTrajectoryCommand.
     *
     * @param driveSubsystem The swerve drive subsystem.
     * @param startPose      The starting Pose2d.
     * @param waypoints      A list of interior waypoints (can be empty for a direct route).
     * @param endPose        The target Pose2d.
     */
    public SwerveTrajectoryCommand(DriveSubsystem driveSubsystem, Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose) {
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPose, waypoints, endPose, config);

        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        swerveCommand = new SwerveControllerCommand(
            trajectory,
            driveSubsystem::getPose,                // Pose supplier.
            DriveConstants.kDriveKinematics,          // Kinematics.
            xController,
            yController,
            thetaController,
            driveSubsystem::setModuleStates,          // Module state consumer.
            driveSubsystem                          // Required subsystem.
        );
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        swerveCommand.initialize();
    }

    @Override
    public void execute() {
        swerveCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return swerveCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        swerveCommand.end(interrupted);
    }
}
