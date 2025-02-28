// File: DynamicDriveToTagCommand.java
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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ReefscapeTargetPoses;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class DynamicDriveToTagCommand extends Command {
    private SwerveControllerCommand internalCommand;
    private final DriveSubsystem driveSubsystem;
    private Pose2d targetPose;

    public DynamicDriveToTagCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Determine the target pose based on vision and alliance.
        int tagId = (int) LimelightHelpers.getFiducialID("limelight"); // Ensure this returns a valid ID.
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            switch (tagId) {
                case 6:
                    targetPose = ReefscapeTargetPoses.RED_TAG6_RIGHT;
                    break;
                case 7:
                    targetPose = ReefscapeTargetPoses.RED_TAG7_RIGHT;
                    break;
                case 8:
                    targetPose = ReefscapeTargetPoses.RED_TAG8_RIGHT;
                    break;
                case 9:
                    targetPose = ReefscapeTargetPoses.RED_TAG9_RIGHT;
                    break;
                case 10:
                    targetPose = ReefscapeTargetPoses.RED_TAG10_RIGHT;
                    break;
                case 11:
                    targetPose = ReefscapeTargetPoses.RED_TAG11_RIGHT;
                    break;
                default:
                    targetPose = driveSubsystem.getPose();
                    break;
            }
        } else { // Blue Alliance
            switch (tagId) {
                case 17:
                    targetPose = ReefscapeTargetPoses.BLUE_TAG17_RIGHT;
                    break;
                case 18:
                    targetPose = ReefscapeTargetPoses.BLUE_TAG18_RIGHT;
                    break;
                case 19:
                    targetPose = ReefscapeTargetPoses.BLUE_TAG19_RIGHT;
                    break;
                case 20:
                    targetPose = ReefscapeTargetPoses.BLUE_TAG20_RIGHT;
                    break;
                case 21:
                    targetPose = ReefscapeTargetPoses.BLUE_TAG21_RIGHT;
                    break;
                case 22:
                    targetPose = ReefscapeTargetPoses.BLUE_TAG22_RIGHT;
                    break;
                default:
                    targetPose = driveSubsystem.getPose();
                    break;
            }
        }

        // Build trajectory using current odometry.
        Pose2d startPose = driveSubsystem.getPose();
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            startPose,
            List.of(),  // Direct path; add interior waypoints if needed.
            targetPose,
            config
        );

        // Create controllers.
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
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

        // Reset odometry to the starting pose.
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
        driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public Set<edu.wpi.first.wpilibj2.command.Subsystem> getRequirements() {
        return internalCommand != null ? internalCommand.getRequirements() : Set.of();
    }

    @Override
    public String getName() {
        return "DynamicDriveToTagCommand";
    }
}
