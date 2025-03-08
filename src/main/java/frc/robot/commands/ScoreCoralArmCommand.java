// ManualCoralArmAdjustCommand.java
// Control Freaks 2025 – Command for manual adjustments to the coral arm

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewCoralArmSubsystem;


public class ScoreCoralArmCommand extends Command {
    private final NewCoralArmSubsystem m_armSubsystem;

    /**
     * Constructs a command for manual coral arm adjustment.
     * @param armSubsystem The coral arm subsystem.
     */

    public ScoreCoralArmCommand(NewCoralArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void execute() {
        // Command the subsystem using the manual adjustment methods.
        m_armSubsystem.manualAdjustArmAngle(0.1);
        m_armSubsystem.manualAdjustArmExtension(0.1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopArm();
    }
}
