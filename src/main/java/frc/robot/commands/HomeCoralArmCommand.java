// HomeCoralArmCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.NewCoralArmSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HomeCoralArmCommand extends Command {
    private final NewCoralArmSubsystem m_armSubsystem;
    private static final double POSITION_TOLERANCE = 0.2; // acceptable error
    private static final double CURRENT_THRESHOLD = 10.0; // amperes for hard stop

    /**
     * Constructs a command to retract the coral arm to its home position.
     *
     * @param armSubsystem The coral arm subsystem.
     */
    public HomeCoralArmCommand(NewCoralArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        // Optionally reset integrators here.
    }

    @Override
    public void execute() {
        // Command the arm to move to home (0 position).
        m_armSubsystem.setArmAngle(ArmConstants.TGT_INIT);
        m_armSubsystem.moveArm(ArmConstants.TGT_INIT);

        // Publish current measurements for debugging.
        double angleCurrent = m_armSubsystem.getAngleCurrent();
        double extendCurrent = m_armSubsystem.getExtendCurrent();
        SmartDashboard.putNumber("Arm Angle Current", angleCurrent);
        SmartDashboard.putNumber("Arm Extend Current", extendCurrent);
    }

    @Override
    public boolean isFinished() {
        boolean hardStop = (m_armSubsystem.getAngleCurrent() > CURRENT_THRESHOLD ||
                              m_armSubsystem.getExtendCurrent() > CURRENT_THRESHOLD);
        double currentExtension = m_armSubsystem.getCurrentExtension();
        boolean nearHome = (Math.abs(currentExtension - ArmConstants.TGT_INIT) < POSITION_TOLERANCE);
        return hardStop && nearHome;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopArm();
    }
}
