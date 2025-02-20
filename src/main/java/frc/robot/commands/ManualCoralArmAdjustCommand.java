// ManualCoralArmAdjustCommand.java
// Control Freaks 2025 – Command for manual adjustments to the coral arm

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewCoralArmSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class ManualCoralArmAdjustCommand extends Command {
    private final NewCoralArmSubsystem m_armSubsystem;
    private final XboxController m_mechController;
    
    // Define a deadband for joystick inputs.
    private static final double DEADBAND = 0.1;
    // Scaling factors for manual adjustments.
    private static final double ANGLE_SCALING = 0.10;
    private static final double EXTENSION_SCALING = 0.50;

    /**
     * Constructs a command for manual coral arm adjustment.
     * @param armSubsystem The coral arm subsystem.
     * @param mechController The mechanism Xbox controller.
     */
    public ManualCoralArmAdjustCommand(NewCoralArmSubsystem armSubsystem, XboxController mechController) {
        m_armSubsystem = armSubsystem;
        m_mechController = mechController;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void execute() {
        // Read joystick values.
        double leftY  = m_mechController.getLeftY();
        double rightY = m_mechController.getRightY();

        // Apply deadband.
        leftY  = Math.abs(leftY)  < DEADBAND ? 0 : leftY;
        rightY = Math.abs(rightY) < DEADBAND ? 0 : rightY;

        // Scale the inputs.
        double angleOutput     = leftY  * ANGLE_SCALING;
        double extensionOutput = rightY * EXTENSION_SCALING;

        // Command the subsystem using the manual adjustment methods.
        m_armSubsystem.manualAdjustArmAngle(angleOutput);
        m_armSubsystem.manualAdjustArmExtension(extensionOutput);
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously during teleop.
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopArm();
    }
}
