// File: DriveForwardOneMeterCommand.java
package frc.robot.autonomous;

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

public class DriveForwardOneMeterCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private SwerveControllerCommand internalCommand;
    private Pose2d startPose = null;

    public DriveForwardOneMeterCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Get the starting pose.
        startPose = driveSubsystem.getPose();
        double theta = startPose.getRotation().getRadians();
        // Rotation2d endPose = new Rotation2d(Math.PI*0.5);
        
        // Compute target pose: 1 meter forward in the current heading.
        double targetX = startPose.getTranslation().getX() + Math.cos(theta) * 1.0;
        double targetY = startPose.getTranslation().getY() + Math.sin(theta) * 1.0;
        
        // Keep the target pose's heading the same for now.
        Pose2d targetPose = new Pose2d(targetX, targetY, startPose.getRotation());
        // Pose2d targetPose = new Pose2d(targetX, targetY, endPose);

    
        // Reset odometry immediately
        driveSubsystem.resetOdometry(startPose);
    
        // Configure trajectory settings.
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);
        
        // Generate the trajectory.
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            startPose,
            List.of(), // No interior waypoints: a direct, straight-line path.
            targetPose,
            config
        );
        
        // Create PID controllers for x and y.
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        
        // Create a motion-profiled PID controller for rotation.
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        // // Set the final heading goal (rotate 180 degrees).
        // thetaController.reset(startPose.getRotation().getRadians()); // Ensure reset
        // thetaController.setGoal(startPose.getRotation().getRadians());
    
        // Create the SwerveControllerCommand.
        internalCommand = new SwerveControllerCommand(
            trajectory,
            driveSubsystem::getPose,                   // Pose supplier.
            DriveConstants.kDriveKinematics,           // Drivetrain kinematics.
            xController,
            yController,
            thetaController,
            (moduleStates) -> {
                driveSubsystem.setModuleStates(moduleStates);
            },
            driveSubsystem                                // Required subsystem.
        );
        
        internalCommand.initialize(); // Initialize the internal command
    }
    
    private boolean hasRunOnce = false;

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
        driveSubsystem.drive(0, 0, 0, true);
    }

    @Override
    public Set<edu.wpi.first.wpilibj2.command.Subsystem> getRequirements() {
        return internalCommand != null ? internalCommand.getRequirements() : Set.of();
    }
}
