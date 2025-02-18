// File: ScoreCoral.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;

/**
 * ScoreCoral command performs a scoring sequence:
 * 1. Concurrently extend and adjust the arm to the scoring position.
 * 2. Wait until the robot reaches the target location.
 * 3. In parallel, adjust the arm angle slightly while retracting to a mid position.
 * 4. Finally, return the arm to its home position.
 */
public class ScoreCoral extends SequentialCommandGroup {
    public ScoreCoral(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, CoralArmSubsystem coralArmSubsystem) {
        addCommands(
            // Phase 1: Extend and set arm angle concurrently to scoring position.
            new ParallelCommandGroup(
                new SetCoralArmPositionCommand(coralArmSubsystem, ArmConstants.TGT_HIGH, ArmConstants.highTgtHeight)
            ),
            // Phase 2: Wait until the drive has reached the target location.
            // (Replace the wait command with an appropriate sensor-based condition if available.)
            new WaitCommand(1.0),
            // Phase 3: Adjust the arm angle slightly while retracting concurrently.
            // Here we subtract a small offset (e.g. 0.5) from the scoring angle and extension.
            new ParallelCommandGroup(
                new SetCoralArmPositionCommand(coralArmSubsystem, ArmConstants.TGT_MID - 0.5, ArmConstants.midTgtHeight - 0.5)
            ),
            // Phase 4: Return the arm to its home position (fully retracted and initial angle).
            new SetCoralArmPositionCommand(coralArmSubsystem, ArmConstants.TGT_INIT, 0.0)
        );
    }
}
