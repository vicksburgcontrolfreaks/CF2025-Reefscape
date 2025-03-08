// File: SetAlgaeCollectorPositionCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;

public class InitAlgaeCollectorPositionCommand extends Command {
    private final AlgaeArmSubsystem subsystem;
    private final double targetPosition;
    private static final double POSITION_TOLERANCE = 1.0; // Acceptable error in encoder units.

    public InitAlgaeCollectorPositionCommand(AlgaeArmSubsystem subsystem, double targetPosition) {
        this.subsystem = subsystem;
        this.targetPosition = targetPosition;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // Command the subsystem to move to the target position.
        subsystem.setArmPosition(targetPosition);
    }

    @Override
    public void execute() {
        // Optionally, you can log current position:
        // System.out.println("Current Position: " + subsystem.getPosition());
    }

    @Override
    public boolean isFinished() {
        // Command finishes when the current position is within tolerance of the target.
        return Math.abs(subsystem.getArmPosition() - targetPosition) < POSITION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor when the command ends.
        subsystem.stop();
    }
}
