// File: RunAlgaeCollectorWheelsCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.AlgaeArmSubsystem;

public class RunAlgaeCollectorWheelsCommand extends Command {
    private final AlgaeArmSubsystem m_algaeArmSubsystem;
    private final Timer m_timer = new Timer();
    private final double m_duration; // Duration in seconds
    private final double m_power;    // Motor output (typically between -1 and 1)

    /**
     * Constructs a command to run the algae collector wheels for a specified duration.
     * 
     * @param subsystem The AlgaeArmSubsystem.
     * @param power The motor output to run the wheels (e.g., 0.2 for 20% power).
     * @param duration The duration (in seconds) to run the wheels.
     */
    public RunAlgaeCollectorWheelsCommand(AlgaeArmSubsystem subsystem, double power, double duration) {
        m_algaeArmSubsystem = subsystem;
        m_power = power;
        m_duration = duration;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_algaeArmSubsystem.setWheelMotor(m_power);
    }

    @Override
    public void execute() {
        // No additional code needed while running; the wheels are set.
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration);
    }

    @Override
    public void end(boolean interrupted) {
        m_algaeArmSubsystem.setWheelMotor(0.0);
        m_timer.stop();
    }
}
