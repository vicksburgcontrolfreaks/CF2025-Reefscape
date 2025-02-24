// File: DriveToTagCommand.java
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final Pose2d targetPose;
    
    // Create PID controllers for X, Y, and rotational control.
    private final PIDController xController = new PIDController(0.5, 0.0, 0.0);
    private final PIDController yController = new PIDController(0.5, 0.0, 0.0);
    private final PIDController thetaController = new PIDController(1.0, 0.0, 0.0);
    
    public DriveToTagCommand(DriveSubsystem driveSubsystem, Pose2d targetPose) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;
        addRequirements(driveSubsystem);
        // Enable continuous input for rotational controller.
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void initialize() {
        // Reset PID controllers.
        xController.reset();
        yController.reset();
        thetaController.reset();
    }
    
    @Override
    public void execute() {
        // Retrieve the current robot pose.
        Pose2d currentPose = driveSubsystem.getPose();
        
        // Compute PID outputs for each component.
        double xSpeed = xController.calculate(currentPose.getTranslation().getX(), targetPose.getTranslation().getX());
        double ySpeed = yController.calculate(currentPose.getTranslation().getY(), targetPose.getTranslation().getY());
        double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        
        // Command the drive subsystem; adjust scaling if necessary.
        driveSubsystem.drive(xSpeed, ySpeed, thetaSpeed, true);
    }
    
    @Override
    public boolean isFinished() {
        // Check if the robot is within a small tolerance of the target.
        Pose2d currentPose = driveSubsystem.getPose();
        double xError = Math.abs(currentPose.getTranslation().getX() - targetPose.getTranslation().getX());
        double yError = Math.abs(currentPose.getTranslation().getY() - targetPose.getTranslation().getY());
        double thetaError = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());
        return (xError < 0.1 && yError < 0.1 && thetaError < 0.1);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot.
        driveSubsystem.drive(0, 0, 0, false);
    }
}
