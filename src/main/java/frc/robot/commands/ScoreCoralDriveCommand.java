// On command by the mech operator, this commend drives to the nearest reef scoring position based on visible
// AprilTag. The offset position from the AprilTag is preprogrammed in the ReefscapeTargetPoses class. 
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ReefscapeTargetPoses;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.List;

public class ScoreCoralDriveCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private SwerveControllerCommand m_trajectoryCommand;
    private boolean m_validMeasurement = true;
    private final boolean m_isLeft; // true for left target; false for right target

    /**
     * Constructs a ScoreCoralDriveCommand.
     *
     * @param driveSubsystem  The drive subsystem.
     * @param visionSubsystem The vision subsystem.
     * @param isLeft          True for left scoring pose; false for right.
     */
    public ScoreCoralDriveCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, boolean isLeft) {
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        m_isLeft = isLeft;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        // Retrieve vision measurement from MegaTag2.
        LimelightHelpers.PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (mt2Estimate.tagCount == 0) {
            m_validMeasurement = false;
            System.out.println("No AprilTag detected. Command will not move the robot.");
            return;
        }
        m_validMeasurement = true;
        
        // Retrieve tag ID from network table
        int detectedTagID = m_visionSubsystem.getDetectedTagIDFromNT();
        
        // Determine alliance.
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        
        // Look up the predetermined target pose from ReefscapeTargetPoses.
        Pose2d destinationPose = null;
        if (alliance == DriverStation.Alliance.Red) {
            switch (detectedTagID) {
                case 6:  destinationPose = m_isLeft ? ReefscapeTargetPoses.RED_TAG6_LEFT  : ReefscapeTargetPoses.RED_TAG6_RIGHT;  break;
                case 7:  destinationPose = m_isLeft ? ReefscapeTargetPoses.RED_TAG7_LEFT  : ReefscapeTargetPoses.RED_TAG7_RIGHT;  break;
                case 8:  destinationPose = m_isLeft ? ReefscapeTargetPoses.RED_TAG8_LEFT  : ReefscapeTargetPoses.RED_TAG8_RIGHT;  break;
                case 9:  destinationPose = m_isLeft ? ReefscapeTargetPoses.RED_TAG9_LEFT  : ReefscapeTargetPoses.RED_TAG9_RIGHT;  break;
                case 10: destinationPose = m_isLeft ? ReefscapeTargetPoses.RED_TAG10_LEFT : ReefscapeTargetPoses.RED_TAG10_RIGHT; break;
                case 11: destinationPose = m_isLeft ? ReefscapeTargetPoses.RED_TAG11_LEFT : ReefscapeTargetPoses.RED_TAG11_RIGHT; break;
                default:
                    System.out.println("Detected tag " + detectedTagID + " is not valid for red alliance scoring.");
                    m_validMeasurement = false;
                    return;
            }
        } else if (alliance == DriverStation.Alliance.Blue) {
            switch (detectedTagID) {
                case 17: destinationPose = m_isLeft ? ReefscapeTargetPoses.BLUE_TAG17_LEFT : ReefscapeTargetPoses.BLUE_TAG17_RIGHT; break;
                case 18: destinationPose = m_isLeft ? ReefscapeTargetPoses.BLUE_TAG18_LEFT : ReefscapeTargetPoses.BLUE_TAG18_RIGHT; break;
                case 19: destinationPose = m_isLeft ? ReefscapeTargetPoses.BLUE_TAG19_LEFT : ReefscapeTargetPoses.BLUE_TAG19_RIGHT; break;
                case 20: destinationPose = m_isLeft ? ReefscapeTargetPoses.BLUE_TAG20_LEFT : ReefscapeTargetPoses.BLUE_TAG20_RIGHT; break;
                case 21: destinationPose = m_isLeft ? ReefscapeTargetPoses.BLUE_TAG21_LEFT : ReefscapeTargetPoses.BLUE_TAG21_RIGHT; break;
                case 22: destinationPose = m_isLeft ? ReefscapeTargetPoses.BLUE_TAG22_LEFT : ReefscapeTargetPoses.BLUE_TAG22_RIGHT; break;
                default:
                    System.out.println("Detected tag " + detectedTagID + " is not valid for blue alliance scoring.");
                    m_validMeasurement = false;
                    return;
            }
        }

        // Generate trajectory from the current pose to the destination pose.
        Pose2d currentPose = m_driveSubsystem.getPose();
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.DriveConstants.kDriveKinematics);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                currentPose,
                List.of(), // no intermediate waypoints
                destinationPose,
                config
        );

        // Create a ProfiledPIDController for theta control.
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

        // Reset odometry to the starting pose.
        m_driveSubsystem.resetOdometry(currentPose);
        m_trajectoryCommand.initialize();
    }

    @Override
    public void execute() {
        if (m_validMeasurement && m_trajectoryCommand != null) {
            m_trajectoryCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if (!m_validMeasurement) {
            return true;
        }
        return m_trajectoryCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (m_validMeasurement && m_trajectoryCommand != null) {
            m_trajectoryCommand.end(interrupted);
        }
        m_driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public String getName() {
        return "ScoreCoralDriveCommand";
    }
}
