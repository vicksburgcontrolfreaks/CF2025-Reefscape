// File: CollectBallCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;

public class CollectBallCommand extends Command {
    private final AlgaeArmSubsystem subsystem;
    private boolean ballCollected;

    public CollectBallCommand(AlgaeArmSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ballCollected = false;
        subsystem.resetState();  // Reset state so the command starts fresh.
    }

    @Override
    public void execute() {
        ballCollected = subsystem.algaeArmCollect();
    }

    @Override
    public boolean isFinished() {
        return ballCollected;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }
}
