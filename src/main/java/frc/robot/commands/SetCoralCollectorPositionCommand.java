// File: SetCoralCollectorPositionCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralCollectorSubsystem;

public class SetCoralCollectorPositionCommand extends Command {
    private final CoralCollectorSubsystem collectorSubsystem;
    private final double targetPosition;
    
    public SetCoralCollectorPositionCommand(CoralCollectorSubsystem subsystem, double targetPosition) {
        this.collectorSubsystem = subsystem;
        this.targetPosition = targetPosition;
        addRequirements(subsystem);
    }
    
    @Override
    public void initialize() {
        // Optionally, reset any state if needed.
    }
    
    @Override
    public void execute() {
        collectorSubsystem.setPosition(targetPosition);
    }
    
    @Override
    public boolean isFinished() {
        // Optionally, finish when the error is within an acceptable threshold.
        // For now, we return false so the command keeps running.
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        collectorSubsystem.stop();
    }
}
