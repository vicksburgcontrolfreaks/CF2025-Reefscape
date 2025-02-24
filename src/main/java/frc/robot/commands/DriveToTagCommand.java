// File: DriveToTagCommand.java
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final Pose2d targetPose;
    
    // PID controllers using gains from AutoConstants.
    private final PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    private final PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    
    public DriveToTagCommand(DriveSubsystem driveSubsystem, Pose2d targetPose) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;
        addRequirements(driveSubsystem);
        
        // Enable continuous input for the rotation controller.
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        // Reset theta controller with the current heading and zero velocity.
        thetaController.reset(new TrapezoidProfile.State(driveSubsystem.getPose().getRotation().getRadians(), 0));
    }
    
    @Override
    public void execute() {
        // Retrieve the current robot pose.
        Pose2d currentPose = driveSubsystem.getPose();
        
        // Calculate speeds using the PID controllers.
        double xSpeed = xController.calculate(currentPose.getTranslation().getX(), targetPose.getTranslation().getX());
        double ySpeed = yController.calculate(currentPose.getTranslation().getY(), targetPose.getTranslation().getY());
        double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        
        // Clamp the speeds based on your maximum speed constants.
        xSpeed = MathUtil.clamp(xSpeed, -AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxSpeedMetersPerSecond);
        ySpeed = MathUtil.clamp(ySpeed, -AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxSpeedMetersPerSecond);
        thetaSpeed = MathUtil.clamp(thetaSpeed, -AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecond);
        
        // Command the drive subsystem.
        driveSubsystem.drive(xSpeed, ySpeed, thetaSpeed, true);
    }
    
    @Override
    public boolean isFinished() {
        // Retrieve the current robot pose.
        Pose2d currentPose = driveSubsystem.getPose();
        double xError = Math.abs(currentPose.getTranslation().getX() - targetPose.getTranslation().getX());
        double yError = Math.abs(currentPose.getTranslation().getY() - targetPose.getTranslation().getY());
        double thetaError = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());
        
        // Return true when errors are within tolerance.
        return (xError < AutoConstants.kForwardTolerance &&
                yError < AutoConstants.kForwardTolerance &&
                thetaError < 0.1);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the drive subsystem.
        driveSubsystem.drive(0, 0, 0, false);
    }
}
