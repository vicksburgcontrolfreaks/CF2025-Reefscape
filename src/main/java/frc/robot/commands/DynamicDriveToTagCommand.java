// File: DynamicDriveToTagCommand.java
package frc.robot.commands;

import java.util.List;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ReefscapeTargetPoses;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;

public class DynamicDriveToTagCommand extends Command {
    private SwerveControllerCommand internalCommand;
    private final DriveSubsystem driveSubsystem;
    private final LocalizationSubsystem localizationSubsystem;
    private Pose2d targetPose;
    private final boolean m_isLeft; // true for left target; false for right target


    public DynamicDriveToTagCommand(DriveSubsystem driveSubsystem, LocalizationSubsystem localizationSubsystem, boolean isLeft) {
        this.driveSubsystem = driveSubsystem;
        this.localizationSubsystem = localizationSubsystem;
        m_isLeft = isLeft;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Determine the target pose based on vision and alliance.
        int tagId = (int) LimelightHelpers.getFiducialID("limelight"); // Ensure this returns a valid ID.
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue) {
            // Blue Alliance
            switch (tagId) {
                case 17: targetPose = m_isLeft ? ReefscapeTargetPoses.BLUE_TAG17_LEFT : ReefscapeTargetPoses.BLUE_TAG17_RIGHT; break;
                case 18: targetPose = m_isLeft ? ReefscapeTargetPoses.BLUE_TAG18_LEFT : ReefscapeTargetPoses.BLUE_TAG18_RIGHT; break;
                case 19: targetPose = m_isLeft ? ReefscapeTargetPoses.BLUE_TAG19_LEFT : ReefscapeTargetPoses.BLUE_TAG19_RIGHT; break;
                case 20: targetPose = m_isLeft ? ReefscapeTargetPoses.BLUE_TAG20_LEFT : ReefscapeTargetPoses.BLUE_TAG20_RIGHT; break;
                case 21: targetPose = m_isLeft ? ReefscapeTargetPoses.BLUE_TAG21_LEFT : ReefscapeTargetPoses.BLUE_TAG21_RIGHT; break;
                case 22: targetPose = m_isLeft ? ReefscapeTargetPoses.BLUE_TAG22_LEFT : ReefscapeTargetPoses.BLUE_TAG22_RIGHT; break;
                default:
                    targetPose = driveSubsystem.getPose().plus(
                            new Transform2d(new Translation2d(-1.0, 0.0), new Rotation2d(0)));
                    break;
            }
        } else { // Red Alliance
            switch (tagId) {
                case 6:  targetPose = m_isLeft ? ReefscapeTargetPoses.RED_TAG6_LEFT  : ReefscapeTargetPoses.RED_TAG6_RIGHT;  break;
                case 7:  targetPose = m_isLeft ? ReefscapeTargetPoses.RED_TAG7_LEFT  : ReefscapeTargetPoses.RED_TAG7_RIGHT;  break;
                case 8:  targetPose = m_isLeft ? ReefscapeTargetPoses.RED_TAG8_LEFT  : ReefscapeTargetPoses.RED_TAG8_RIGHT;  break;
                case 9:  targetPose = m_isLeft ? ReefscapeTargetPoses.RED_TAG9_LEFT  : ReefscapeTargetPoses.RED_TAG9_RIGHT;  break;
                case 10: targetPose = m_isLeft ? ReefscapeTargetPoses.RED_TAG10_LEFT : ReefscapeTargetPoses.RED_TAG10_RIGHT; break;
                case 11: targetPose = m_isLeft ? ReefscapeTargetPoses.RED_TAG11_LEFT : ReefscapeTargetPoses.RED_TAG11_RIGHT; break;
                default:
                    targetPose = driveSubsystem.getPose().plus(
                            new Transform2d(new Translation2d(1.0, 0.0), new Rotation2d(0)));
                    break;
            }
        }

        // Build trajectory using the current estimated pose from localization.
        Pose2d startPose = localizationSubsystem.getEstimatedPose();
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                startPose,
                List.of(), // Direct path; add interior waypoints if needed.
                targetPose,
                config);

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
                driveSubsystem);

        // Reset odometry to the starting pose.
        driveSubsystem.resetOdometry(startPose);
        internalCommand.initialize();
        SmartDashboard.putString("Trajectory Start Pose", startPose.toString());
        SmartDashboard.putString("Trajectory Target Pose", targetPose.toString());
    }

    @Override
    public void execute() {
        if (internalCommand != null) {
            internalCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        // Use a 6-inch tolerance (~0.1524 m) for translation and 3 degrees (~0.0524
        // rad) for heading.
        Pose2d currentPose = driveSubsystem.getPose();
        double xError = Math.abs(currentPose.getTranslation().getX() - targetPose.getTranslation().getX());
        double yError = Math.abs(currentPose.getTranslation().getY() - targetPose.getTranslation().getY());
        double translationalError = Math.hypot(xError, yError);
        double thetaError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

        return (translationalError < AutoConstants.kForwardTolerance
                && thetaError < Math.toRadians(AutoConstants.kLateralTolerance));
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
