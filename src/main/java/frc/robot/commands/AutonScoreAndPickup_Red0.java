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
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.commands.InitAlgaeCollectorPositionCommand;

public class AutonScoreAndPickup_Red0 extends SequentialCommandGroup {
    public AutonScoreAndPickup_Red0(DriveSubsystem driveSubsystem,
                                   LocalizationSubsystem localizationSubsystem,
                                   VisionSubsystem visionSubsystem,
                                   NewCoralArmSubsystem coralArmSubsystem,
								   AlgaeArmSubsystem algaeArmSubsystem) {
        addCommands(
            // 1. Initialize localization.
            new InitializeLocalizationCommand(driveSubsystem, localizationSubsystem),

            // 2. Drive to the scoring location
            new ParallelCommandGroup(
                new DriveToPoseCommand(driveSubsystem, ReefscapeTargetPoses.RED_TAG11_RIGHT, localizationSubsystem)
            ),
            //InitAlgaeCollectorPositionCommand
            // 3. Score the preloaded coral while driving to the pickup location.
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.1), // Delay 0.5 seconds before starting arm set.
                    new InitAlgaeCollectorPositionCommand(algaeArmSubsystem, 20),
                    new WaitCommand(0.1),
                    new RunAlgaeCollectorWheelsCommand(algaeArmSubsystem, 0.5, 0.75),
                    new WaitCommand(0.1)
                )
            ),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.1)
                )
            ),
            // 4. Back up from the reef to find a location to turn and drive to 
            // new ParallelCommandGroup(
            //     new DriveToPoseCommand(driveSubsystem, ReefscapeTargetPoses.RED_0_INT, localizationSubsystem)
            // ),

            // 5. drive to coral station
            new ParallelCommandGroup(
                new DriveToPoseCommand(driveSubsystem, ReefscapeTargetPoses.RED_CORAL_STATION_0, 
                List.of(ReefscapeTargetPoses.RED_0_INTA,
                        ReefscapeTargetPoses.RED_0_INTB), localizationSubsystem)
            ),

            // 6. Wait for coral to be deposited
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(5.0)
                )
            ),

            //  7. Drive back up to reef 
            new ParallelCommandGroup(
                new DriveToPoseCommand(driveSubsystem, ReefscapeTargetPoses.RED_TAG6_LEFT, localizationSubsystem),
                new SequentialCommandGroup(
                    new WaitCommand(0.1), // Delay 0.5 seconds before starting arm set.
                    new SetArmPositionCommand(coralArmSubsystem, ArmConstants.highTgtAngle, ArmConstants.highTgtHeight)
                )
            ),

            // 8. Score high 
            new ParallelCommandGroup(
                new WaitCommand(1.0),
                new HighScoringSequenceCommand(coralArmSubsystem),
                new WaitCommand(2.0)
            )
        );
    }
}
