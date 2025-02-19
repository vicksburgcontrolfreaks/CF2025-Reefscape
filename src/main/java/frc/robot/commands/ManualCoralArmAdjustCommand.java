// File: ManualCoralArmAdjustCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArmSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class ManualCoralArmAdjustCommand extends Command {
    private final CoralArmSubsystem m_armSubsystem;
    private final XboxController m_mechController;
    // Define a deadband for joystick inputs.
    private static final double DEADBAND = 0.1;
    // Optionally define scaling factors.
    private static final double ANGLE_SCALING = 0.10;
    private static final double EXTENSION_SCALING = 0.50;

    /**
     * Constructs a command for manual coral arm adjustment.
     * @param armSubsystem The coral arm subsystem.
     * @param mechController The mechanism Xbox controller.
     */
    public ManualCoralArmAdjustCommand(CoralArmSubsystem armSubsystem, XboxController mechController) {
        m_armSubsystem = armSubsystem;
        m_mechController = mechController;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void execute() {
        // Read joystick values.
        double leftY = m_mechController.getLeftY();
        double rightY = m_mechController.getRightY();

        // Apply deadband.
        leftY = Math.abs(leftY) < DEADBAND ? 0 : leftY;
        rightY = Math.abs(rightY) < DEADBAND ? 0 : rightY;

        // Scale inputs if necessary.
        double angleOutput = leftY * ANGLE_SCALING;
        double extensionOutput = rightY * EXTENSION_SCALING;

        // Command the subsystem using the manual adjustment methods.
        m_armSubsystem.manualAdjustArmAngle(angleOutput);
        m_armSubsystem.manualAdjustArmExtension(extensionOutput);
    }

    @Override
    public boolean isFinished() {
        return false; // Run continuously until interrupted.
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopArm();
    }
}
