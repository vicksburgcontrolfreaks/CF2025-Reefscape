package frc.robot.autonomous;

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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.List;
import java.util.Set;

public class TrajectoryAutoCommand extends Command {
    private final SwerveControllerCommand internalCommand;
    private final DriveSubsystem driveSubsystem;

    public TrajectoryAutoCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        // Create config for trajectory.
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        // Generate an example trajectory.
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                // Start pose, start on the red side facing directly at tag 10, robot center exactly 6 feet from tag 
                new Pose2d(10, 4, new Rotation2d(0)),
                // Pass through interior waypoints.
                List.of(),
                // Scoring location for tag 10, right.
                new Pose2d(11.829, 4.210, new Rotation2d(Math.toRadians(0))),
                // Constants.ReefscapeTargetPoses.RED_TAG10_RIGHT,
                config);

        // Create a theta controller for the robot's orientation.
        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Create the SwerveControllerCommand that follows the trajectory.
        internalCommand = new SwerveControllerCommand(
                trajectory,
                driveSubsystem::getPose, // Supplier for the robot's pose.
                DriveConstants.kDriveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                driveSubsystem::setModuleStates, // Method to set the module states.
                driveSubsystem);

        // Reset the robot odometry to the starting pose of the trajectory.
        driveSubsystem.resetOdometry(trajectory.getInitialPose());
    }

    @Override
    public void initialize() {
        internalCommand.initialize();
    }

    @Override
    public void execute() {
        internalCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return internalCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        internalCommand.end(interrupted);
        // Stop the drivetrain after completing the command.
        driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return internalCommand.getRequirements();
    }

    @Override
    public String getName() {
        return "TrajectoryAutoCommand";
    }
}
