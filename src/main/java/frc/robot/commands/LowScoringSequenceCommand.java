// File: LowScoringSequenceCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.NewCoralArmSubsystem;

public class LowScoringSequenceCommand extends SequentialCommandGroup {
    /**
     * Constructs the low-target scoring sequence command.
     * The sequence is:
     *   1. Set arm angle to 44.
     *   2. Wait 0.5 seconds.
     *   3. Set arm extension to -24.
     *   4. Wait 0.5 seconds.
     *   5. Set arm angle to 30.
     *   6. Execute HomeCoralArmCommand to retract/park the arm.
     *
     * @param armSubsystem The NewCoralArmSubsystem controlling the arm.
     */
    public LowScoringSequenceCommand(NewCoralArmSubsystem armSubsystem) {
        addCommands(
            new InstantCommand(() -> armSubsystem.setArmAngle(44), armSubsystem),
            new WaitCommand(0.5),
            new InstantCommand(() -> armSubsystem.moveArm(-24), armSubsystem),
            new WaitCommand(0.5),
            new InstantCommand(() -> armSubsystem.setArmAngle(35), armSubsystem),
            new WaitCommand(0.5),
            new InstantCommand(() -> armSubsystem.moveArm(-5), armSubsystem),
            new WaitCommand(0.5),
            new InstantCommand(() -> armSubsystem.setArmAngle(25), armSubsystem),
            new WaitCommand(0.5),
            new HomeCoralArmCommand(armSubsystem)
        );
    }
}
