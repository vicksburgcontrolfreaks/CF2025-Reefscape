package frc.robot.commands;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.LimelightHelpers;

public class DriveToPoseCommand extends Command {
    private SwerveControllerCommand internalCommand;
    private final DriveSubsystem driveSubsystem;
    private final Pose2d targetPose;
    private final List<Pose2d> interiorWaypoints;
    private final LocalizationSubsystem localizationSubsystem;

    /**
     * Creates a new DriveToPoseCommand that will generate a trajectory from the current
     * pose to the target pose using the provided interior waypoints.
     *
     * @param driveSubsystem  The drive subsystem.
     * @param targetPose      The final target pose.
     * @param interiorWaypoints A list of interior waypoints (as Pose2d objects) for the trajectory.
     */
    public DriveToPoseCommand(DriveSubsystem driveSubsystem, Pose2d targetPose, List<Pose2d> interiorWaypoints, LocalizationSubsystem localizationSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;
        this.interiorWaypoints = interiorWaypoints;
        this.localizationSubsystem = localizationSubsystem;
        addRequirements(driveSubsystem);
    }

    /**
     * Overloaded constructor that assumes no interior waypoints.
     *
     * @param driveSubsystem The drive subsystem.
     * @param targetPose     The final target pose.
     */
    public DriveToPoseCommand(DriveSubsystem driveSubsystem, Pose2d targetPose, LocalizationSubsystem localizationSubsystem) {
        this(driveSubsystem, targetPose, List.of(), localizationSubsystem);
    }

    @Override
    public void initialize() {
        // Get the current odometry as the starting pose.
        Pose2d startPose = localizationSubsystem.getEstimatedPose();
        // Check if a vision estimate is available.
        var visionEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (visionEstimate != null && visionEstimate.tagCount > 0) {
            startPose = visionEstimate.pose;
        }

        // Convert interior waypoints (Pose2d) to a list of Translation2d.
        List<Translation2d> translations = interiorWaypoints.stream()
                .map(Pose2d::getTranslation)
                .collect(Collectors.toList());

        // Build a trajectory from startPose to targetPose using the interior waypoints.
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                startPose,
                translations,
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
        // Stop the drive subsystem.
        driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public Set<edu.wpi.first.wpilibj2.command.Subsystem> getRequirements() {
        return internalCommand != null ? internalCommand.getRequirements() : Set.of();
    }
}
