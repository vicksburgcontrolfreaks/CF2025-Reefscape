package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.NewCoralArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ReefscapeTargetPoses;

public class AutonScoreAndPickup_Red0 extends SequentialCommandGroup {
    public AutonScoreAndPickup_Red0(DriveSubsystem driveSubsystem,
                                   LocalizationSubsystem localizationSubsystem,
                                   VisionSubsystem visionSubsystem,
                                   NewCoralArmSubsystem coralArmSubsystem) {
        addCommands(
            // 1. Initialize localization.
            new InitializeLocalizationCommand(driveSubsystem, localizationSubsystem),

            // 2. Drive to the scoring location with preloaded gamepiece while concurrently setting the arm.
            new ParallelCommandGroup(
                // new DriveToPoseCommand(driveSubsystem, ReefscapeTargetPoses.RED_TAG11_LEFT),
                new DriveToPoseCommand(driveSubsystem, ReefscapeTargetPoses.RED_TAG11_LEFT, List.of(new Pose2d(12.5, 2.73, new Rotation2d(Math.toRadians(60))))),

                new SequentialCommandGroup(
                    new WaitCommand(0.1), // Delay 0.5 seconds before starting arm set.
                    new SetArmPositionCommand(coralArmSubsystem, ArmConstants.highTgtAngle, ArmConstants.highTgtHeight)
                )
            ),

            // 3. Score the preloaded coral while driving to the pickup location.
            new ParallelCommandGroup(
                new HighScoringSequenceCommand(coralArmSubsystem),
                new SequentialCommandGroup(
                    new WaitCommand(2.0), // Delay for the arm to clear the reef
                    new DriveToPoseCommand(driveSubsystem, ReefscapeTargetPoses.RED_CORAL_STATION_0),
                    new WaitCommand(2.0)  // Delay to allow human player to load coral
                )
            // ),

            // // 4. Return to the reef while concurrently setting the arm for scoring.
            // new ParallelCommandGroup(
            //     new DriveToPoseCommand(driveSubsystem, ReefscapeTargetPoses.RED_TAG6_LEFT),
            //     new SequentialCommandGroup(
            //         new WaitCommand(0.5), // Delay before starting arm set.
            //         new SetArmPositionCommand(coralArmSubsystem, ArmConstants.highTgtAngle, ArmConstants.highTgtHeight)
                )
            // ),

            // // 5. Score the second coral.
            // new HighScoringSequenceCommand(coralArmSubsystem)
        );
    }
}
