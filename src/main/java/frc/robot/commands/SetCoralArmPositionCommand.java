// File: SetCoralArmPositionCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArmSubsystem;

public class SetCoralArmPositionCommand extends Command {
    private final CoralArmSubsystem m_armSubsystem;
    private final double m_targetAngle;
    private final double m_targetExtension;
    // Optionally, define tolerances if you want to finish when the arm is near its target.
    private static final double ANGLE_TOLERANCE = 0.1;
    private static final double EXTENSION_TOLERANCE = 0.1;

    /**
     * Constructs a command that sets the coral arm to a specific angle and extension.
     *
     * @param armSubsystem     The coral arm subsystem.
     * @param targetAngle      The desired arm angle (in your subsystem’s units, e.g., encoder units or degrees).
     * @param targetExtension  The desired extension position.
     */
    public SetCoralArmPositionCommand(CoralArmSubsystem armSubsystem, double targetAngle, double targetExtension) {
        m_armSubsystem = armSubsystem;
        m_targetAngle = targetAngle;
        m_targetExtension = targetExtension;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        // Optionally, reset any integrators or initialize state if needed.
    }

    @Override
    public void execute() {
        // Command the subsystem to move the arm angle and extension.
        m_armSubsystem.setArmAngle((int)m_targetAngle);  // cast if your method still expects an int
        m_armSubsystem.moveArm((int)m_targetExtension);   // cast if necessary
    }

    @Override
    public boolean isFinished() {
        // If you have getters in your subsystem, you could compare the current positions to the targets:
        // double currentAngle = m_armSubsystem.getArmAngle();
        // double currentExtension = m_armSubsystem.getExtension();
        // return (Math.abs(currentAngle - m_targetAngle) < ANGLE_TOLERANCE) &&
        //        (Math.abs(currentExtension - m_targetExtension) < EXTENSION_TOLERANCE);
        // Otherwise, if it's a one-shot command, return true immediately.
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopArm();
    }
}
