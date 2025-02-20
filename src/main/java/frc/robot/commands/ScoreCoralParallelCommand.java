// File: ScoreCoralParallelCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;

public class ScoreCoralParallelCommand extends ParallelDeadlineGroup {
    /**
     * Constructs a ScoreCoralParallelCommand that drives the robot to score coral while
     * concurrently operating the coral arm.
     *
     * @param driveSubsystem    The drive subsystem.
     * @param visionSubsystem   The vision subsystem.
     * @param coralArmSubsystem The coral arm subsystem.
     * @param scoringSideLeft   True for left scoring position; false for right.
     */
    public ScoreCoralParallelCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, CoralArmSubsystem coralArmSubsystem, boolean scoringSideLeft) {
        super(
            // Deadline command: use ScoreCoralDriveCommand with the desired scoring side.
            new ScoreCoralDriveCommand(driveSubsystem, visionSubsystem, scoringSideLeft),
            // Parallel mechanism commands: raise the arm, wait briefly, then release coral.
            new SequentialCommandGroup(
                new SetCoralArmPositionCommand(coralArmSubsystem, 1),  // Example: target angle = 1 and extension = 1.0 (adjust as needed)
                new WaitCommand(0.5),
                new ReleaseCoralCmd(coralArmSubsystem, 0.5)      // Example release speed = 0.5 (adjust as needed)
            )
        );
    }
}
