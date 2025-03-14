// File: MidScoringSequenceCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.NewCoralArmSubsystem;

public class HighScoringSequenceCommand extends SequentialCommandGroup {
    /**
     * Constructs the mid-target scoring sequence command.
     * Sequence:
     *   1. Set arm angle to 35 (placeholder value)
     *   2. Wait 0.5 seconds.
     *   3. Set arm extension to -40 (placeholder value)
     *   4. Wait 0.5 seconds.
     *   5. Set arm angle to 25 (placeholder value)
     *   6. Execute HomeCoralArmCommand to retract/park the arm.
     *
     * Adjust these values as needed.
     *
     * @param armSubsystem The NewCoralArmSubsystem controlling the arm.
     */
    public HighScoringSequenceCommand(NewCoralArmSubsystem armSubsystem) {
        addCommands(
            new InstantCommand(() -> armSubsystem.setArmAngle(.14), armSubsystem),
            new WaitCommand(0.2), 
            new InstantCommand(() -> armSubsystem.moveArm(-60), armSubsystem),
            new WaitCommand(0.2),
            new InstantCommand(() -> armSubsystem.setArmAngle(.16), armSubsystem),
            new WaitCommand(0.2),
            new InstantCommand(() -> armSubsystem.moveArm(-10), armSubsystem),
            new WaitCommand(0.2),
            new HomeCoralArmCommand(armSubsystem)
        );
    }
}