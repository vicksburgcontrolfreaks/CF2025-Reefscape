// ManualCoralArmAdjustCommand.java
// Control Freaks 2025 – Command for manual adjustments to the coral arm

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewCoralArmSubsystem;
import frc.robot.Constants.ArmConstants;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj.XboxController;

public class ScoreCoralArmCommand extends Command {
    private final NewCoralArmSubsystem m_armSubsystem;
    private int m_position;
    private int m_position_prev = -1;
    private int counter;

    /**
     * Constructs a command for manual coral arm adjustment.
     * @param armSubsystem The coral arm subsystem.
     */

    public ScoreCoralArmCommand(NewCoralArmSubsystem armSubsystem, int position) {
        m_armSubsystem = armSubsystem;
        m_position = position;
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

        if (m_position != m_position_prev)
        {
            counter = 0;
        }

        counter++;
        
        switch (m_position) {
            case ArmConstants.TGT_LOW:
                if (counter >= 100)
                {
                    return true;
                }
                break;
            case ArmConstants.TGT_MID:
                if (counter >= 150)
                {
                    return true;
                }   
                break;
            case ArmConstants.TGT_HIGH:
                if (counter >= 200)
                {
                    return true;
                }
                break;
        }

        m_position_prev = m_position;

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopArm();
    }
}
