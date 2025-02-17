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
     * concurrently raising and releasing the coral arm.
     *
     * @param driveSubsystem    The drive subsystem.
     * @param visionSubsystem   The vision subsystem.
     * @param CoralArmSubsystem The coral arm subsystem.
     */
    public ScoreCoralParallelCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, CoralArmSubsystem coralArmSubsystem) {
        super(
            // Deadline command: use TrajectoryToTagCommand (with desired offset) to navigate.
            new ScoreCoralDriveCommand(driveSubsystem, visionSubsystem, true),
            // Parallel mechanism commands: Raise arm, wait briefly, then release coral.
            new SequentialCommandGroup(
                new raiseCoralArm(coralArmSubsystem, 1),
                new WaitCommand(0.5),
                new releaseCoralCmd(coralArmSubsystem, 0.5)
                // Optionally, add additional commands (e.g., retract arm) if needed.
            )
        );
    }
}
