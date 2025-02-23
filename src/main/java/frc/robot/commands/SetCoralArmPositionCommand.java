package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArmSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SetCoralArmPositionCommand extends Command {
    private final CoralArmSubsystem m_armSubsystem;
    private final int m_scoringPosition;
    // Define tolerances so we know when the arm is "close enough."
    private static final double ANGLE_TOLERANCE = 0.1;
    private static final double EXTENSION_TOLERANCE = 0.1;
    private int increment = 0;

    public SetCoralArmPositionCommand(CoralArmSubsystem armSubsystem, int scoringPosition) {
        m_armSubsystem = armSubsystem;
        m_scoringPosition = scoringPosition;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        // Optionally reset integrators or other state here.
    }

    @Override
    public void execute() {
        increment ++;
        m_armSubsystem.setArmAngle(m_scoringPosition);  //.setArmPosition(m_scoringPosition);
        //if (m_armSubsystem.getArmAngle() > m_armSubsystem.getInitArmAngle() + 6) {
            m_armSubsystem.moveArm(m_scoringPosition);
        //}
        
        SmartDashboard.putNumber("SetCoralArmCmd Increment", increment);
        //m_armSubsystem.set
        // Alternatively, if you prefer separate commands:
        // m_armSubsystem.setArmAngle(m_targetAngle);
        // m_armSubsystem.moveArm(m_targetExtension);
    }

    @Override
    public boolean isFinished() {
        double currentAngle = m_armSubsystem.getArmAngle();
        double currentExtension = m_armSubsystem.getCurrentExtension();
        //return (Math.abs(currentAngle - m_targetAngle) < ANGLE_TOLERANCE) &&
        //       (Math.abs(currentExtension - m_targetExtension) < EXTENSION_TOLERANCE);
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopArm();
    }

}
