// File: HomeCoralArmCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.CoralArmSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HomeCoralArmCommand extends Command {
    private final CoralArmSubsystem m_armSubsystem;
    // Tolerance for considering the mechanism near the home (zero) position.
    private static final double POSITION_TOLERANCE = 0.2;
    // Current threshold (in Amperes) that indicates a hard stop (adjust based on testing).
    private static final double CURRENT_THRESHOLD = 10.0;

    /**
     * Constructs a command to retract the coral arm to its home position.
     *
     * @param armSubsystem The coral arm subsystem.
     */
    public HomeCoralArmCommand(CoralArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        // Optionally, reset any integrators or state here.
    }

    @Override
    public void execute() {
        // Command the arm to move to the home position (TGT_INIT, assumed to be 0).
        m_armSubsystem.setArmAngle(ArmConstants.TGT_INIT);
        m_armSubsystem.moveArm(ArmConstants.TGT_INIT);

        // Optionally, if you wish to slow down manually near the end,
        // you could incorporate additional logic here or in the subsystem control loops.

        // Publish current measurements for debugging.
        double angleCurrent = m_armSubsystem.getAngleCurrent();   // Implement this in your subsystem.
        double extendCurrent = m_armSubsystem.getExtendCurrent(); // Implement this in your subsystem.
        SmartDashboard.putNumber("Arm Angle Current", angleCurrent);
        SmartDashboard.putNumber("Arm Extend Current", extendCurrent);
    }

    @Override
    public boolean isFinished() {
        // Option 1: If either motor's current spikes above the threshold, consider that we've reached a hard stop.
        boolean hardStop = (m_armSubsystem.getAngleCurrent() > CURRENT_THRESHOLD ||
                            m_armSubsystem.getExtendCurrent() > CURRENT_THRESHOLD);
        
        // Option 2: Additionally, you might check that the extension is within a small tolerance of 0.
        double currentExtension = m_armSubsystem.getCurrentExtension(); // You may implement this getter.
        boolean nearHome = (Math.abs(currentExtension - ArmConstants.TGT_INIT) < POSITION_TOLERANCE);
        
        // We finish if a hard stop is detected and the mechanism is near the home position.
        return hardStop && nearHome;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopArm();
    }
}
