package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArmSubsystem;

public class SetCoralArmPositionCommand extends Command {
    private final CoralArmSubsystem m_armSubsystem;
    private final double m_targetAngle;
    private final double m_targetExtension;
    // Define tolerances so we know when the arm is "close enough."
    private static final double ANGLE_TOLERANCE = 0.1;
    private static final double EXTENSION_TOLERANCE = 0.1;

    public SetCoralArmPositionCommand(CoralArmSubsystem armSubsystem, double targetAngle, double targetExtension) {
        m_armSubsystem = armSubsystem;
        m_targetAngle = targetAngle;
        m_targetExtension = targetExtension;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        // Optionally reset integrators or other state here.
    }

    @Override
    public void execute() {
        // If using the combined control method:
        m_armSubsystem.setArmPosition(m_targetAngle, m_targetExtension, 0.5);
        // Alternatively, if you prefer separate commands:
        // m_armSubsystem.setArmAngle(m_targetAngle);
        // m_armSubsystem.moveArm(m_targetExtension);
    }

    @Override
    public boolean isFinished() {
        double currentAngle = m_armSubsystem.getArmAngle();
        double currentExtension = m_armSubsystem.getCurrentExtension();
        return (Math.abs(currentAngle - m_targetAngle) < ANGLE_TOLERANCE) &&
               (Math.abs(currentExtension - m_targetExtension) < EXTENSION_TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopArm();
    }
}
