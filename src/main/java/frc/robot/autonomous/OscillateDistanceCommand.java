// File: OscillateDistanceCommand.java
package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OscillateDistanceCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final PIDController pid;
    private Pose2d initialPose;
    private double currentSetpoint;
    private final double oscillationDistance = 2.0; // in meters

    /**
     * Creates a command that oscillates the robot between its starting x‑position
     * and a point 2 meters ahead. The proportional gain (kP) is read from SmartDashboard.
     *
     * @param driveSubsystem The drive subsystem used to control the robot.
     */
    public OscillateDistanceCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        // Initialize the PID controller with an initial kP value.
        double initialKp = SmartDashboard.getNumber("Test kP", 0.05);
        pid = new PIDController(initialKp, 0, 0);
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Capture the initial pose when the command starts.
        initialPose = driveSubsystem.getPose();
        // Set the initial setpoint to be 2 meters ahead of the initial x position.
        currentSetpoint = initialPose.getTranslation().getX() + oscillationDistance;
        pid.reset();
        pid.setSetpoint(currentSetpoint);
    }

    @Override
    public void execute() {
        // Optionally update kP dynamically from SmartDashboard.
        double kP = SmartDashboard.getNumber("Test kP", 0.05);
        pid.setP(kP);

        // Get the current x-coordinate from odometry.
        double currentX = driveSubsystem.getPose().getTranslation().getX();
        double output = pid.calculate(currentX, currentSetpoint);

        // Command the drive subsystem to drive forward/backward.
        // Here, we command only x-direction speed (y=0 and rotation=0).
        driveSubsystem.drive(output, 0, 0, true);

        // Check if the error is small enough to toggle the setpoint.
        double error = Math.abs(currentSetpoint - currentX);
        if (error < 0.1) { // Threshold in meters; adjust as needed.
            // Toggle the setpoint:
            double initialX = initialPose.getTranslation().getX();
            if (Math.abs(currentSetpoint - (initialX + oscillationDistance)) < 0.01) {
                // If currently set to forward (initial + 2.0 m), set target back to initial.
                currentSetpoint = initialX;
            } else {
                // Otherwise, set target to initial + 2.0 m.
                currentSetpoint = initialX + oscillationDistance;
            }
            pid.setSetpoint(currentSetpoint);
        }
    }

    @Override
    public boolean isFinished() {
        // Run indefinitely until manually cancelled.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends.
        driveSubsystem.drive(0, 0, 0, false);
    }
}
