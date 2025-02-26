// File: OscillateDistanceCommand.java
package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OscillateDistanceCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final PIDController m_pid;
    private Pose2d m_initialPose;
    private double m_currentSetpoint;
    private double m_tolerance = 0.05; //meters
    private final double m_oscillationDistance = 2.0; // in meters
    private boolean m_movingForward; // true when moving toward m_initialPose + 2.0, false when moving back

    /**
     * Creates a command that oscillates the robot between its starting x‑position
     * and a point 2 meters ahead.
     *
     * @param driveSubsystem The drive subsystem used to control the robot.
     */
    public OscillateDistanceCommand(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        // Get the initial kP value from SmartDashboard
        double initialKp = SmartDashboard.getNumber("Test kP", 0.05);
        m_pid = new PIDController(initialKp, 0, 0);
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        // Capture the starting pose.
        m_initialPose = m_driveSubsystem.getPose();
        // Set initial target to 2m ahead.
        m_currentSetpoint = m_initialPose.getTranslation().getX() + m_oscillationDistance;
        m_pid.reset();
        m_pid.setSetpoint(m_currentSetpoint);
        m_movingForward = true;
    }

    @Override
    public void execute() {
        // Update kP dynamically from SmartDashboard.
        double kP = SmartDashboard.getNumber("Test kP", 0.05);
        m_pid.setP(kP);

        // Get the current X position.
        double currentX = m_driveSubsystem.getPose().getTranslation().getX();
        double output = m_pid.calculate(currentX, m_currentSetpoint);
        // Command the drive subsystem along the X direction.
        m_driveSubsystem.drive(output, 0, 0, true);

        // Check if the error is small enough to toggle.
        double error = Math.abs(m_currentSetpoint - currentX);
        if (error < m_tolerance) {
            // Toggle direction.
            if (m_movingForward) {
                m_currentSetpoint = m_initialPose.getTranslation().getX();
                m_movingForward = false;
            } else {
                m_currentSetpoint = m_initialPose.getTranslation().getX() + m_oscillationDistance;
                m_movingForward = true;
            }
            m_pid.setSetpoint(m_currentSetpoint);
        }

        // Publish diagnostic information.
        SmartDashboard.putNumber("Oscillate Error", error);
        SmartDashboard.putNumber("Oscillate Setpoint", m_currentSetpoint);
    }

    @Override
    public boolean isFinished() {
        return false; // Run indefinitely until cancelled.
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends.
        m_driveSubsystem.drive(0, 0, 0, false);
    }
}
