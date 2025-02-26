// File: ReleaseBallCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;

public class ReleaseBallCommand extends Command {
    private final AlgaeArmSubsystem subsystem;
    private boolean done = false;
    
    public ReleaseBallCommand(AlgaeArmSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }
    
    @Override
    public void initialize() {
        done = false;
    }
    
    @Override
    public void execute() {
        // This method should drive the motor to reverse the wheels to eject the ball
        // and retract the arm until it reaches the zero position.
        done = subsystem.algaeArmShoot();
    }
    
    @Override
    public boolean isFinished() {
        return done;
    }
    
    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
        // Instead of zeroing the encoder, we rely on the mechanism to drive the arm physically to zero.
        subsystem.resetState(); // Reset state variables for future command runs.
    }
}
