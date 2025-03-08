// File: DeployAndStowAlgaeArmCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeArmSubsystem;

public class DeployAndStowAlgaeArmCommand extends Command {
    private final AlgaeArmSubsystem subsystem;
    private final Timer timer = new Timer();
    
    private enum State {
        EXTENDING,      // Command the arm to extend.
        RUNNING_WHEELS, // Run the wheel motor for 500ms.
        RETRACTING,     // Command the arm to retract.
        FINISHED
    }
    private State state = State.EXTENDING;
    
    // Tolerance (encoder units) for determining when the arm has reached its target.
    private static final double POSITION_TOLERANCE = 1.0;
    
    // Use the constant from AlgaeConstants.
    private static final double ARM_COLLECT_TARGET = AlgaeConstants.extended;
    private static final double ARM_READY_TARGET = AlgaeConstants.ready;
    
    public DeployAndStowAlgaeArmCommand(AlgaeArmSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        state = State.EXTENDING;
        timer.reset();
        timer.start();
        // Command the arm to extend to the target defined in AlgaeConstants.
        subsystem.setArmPosition(ARM_COLLECT_TARGET);
    }

    @Override
    public void execute() {
        switch (state) {
            case EXTENDING:
                // Wait until the arm is within tolerance of the target.
                if (Math.abs(subsystem.getArmPosition() - ARM_COLLECT_TARGET) < POSITION_TOLERANCE) {
                    state = State.RUNNING_WHEELS;
                    timer.reset();
                }
                break;
                
            case RUNNING_WHEELS:
                // Run the wheel motor at 20% power.
                subsystem.setWheelMotor(0.20);
                // After 500 ms, stop the wheels and command the arm to retract.
                if (timer.hasElapsed(0.5)) {
                    subsystem.setWheelMotor(0);
                    state = State.RETRACTING;
                    subsystem.setArmPosition(ARM_READY_TARGET); // Command the arm to retract (ready position).
                }
                break;
                
            case RETRACTING:
                // Check if the arm is retracted (within tolerance of zero).
                if (Math.abs(subsystem.getArmPosition()) < POSITION_TOLERANCE) {
                    state = State.FINISHED;
                }
                break;
                
            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        subsystem.stop(); // Stop both the arm and wheel motors.
    }
}
