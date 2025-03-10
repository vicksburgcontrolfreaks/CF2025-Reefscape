package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.NewCoralArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.ArmConstants;

public class TeleopAutoScoreCommand extends SequentialCommandGroup {
    public TeleopAutoScoreCommand(DriveSubsystem driveSubsystem,
            LocalizationSubsystem localizationSubsystem,
            VisionSubsystem visionSubsystem,
            NewCoralArmSubsystem coralArmSubsystem, boolean isLeft) {
        addCommands(
                // 1. Initialize localization.
                new InitializeLocalizationCommand(driveSubsystem, localizationSubsystem),

                // 2. Drive to the scoring location with preloaded gamepiece while concurrently
                // setting the arm.
                new ParallelCommandGroup(
                        new DynamicDriveToTagCommand(driveSubsystem, localizationSubsystem, isLeft),
                        new SequentialCommandGroup(
                                new WaitCommand(0.5), // Delay in seconds before starting arm set.
                                new SetArmPositionCommand(coralArmSubsystem, ArmConstants.highTgtAngle,
                                        ArmConstants.highTgtHeight))));
    }
}
