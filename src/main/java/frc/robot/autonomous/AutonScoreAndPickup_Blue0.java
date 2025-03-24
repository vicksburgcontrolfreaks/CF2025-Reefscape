package frc.robot.autonomous;

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
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.HighScoringSequenceCommand;
import frc.robot.commands.InitAlgaeCollectorPositionCommand;
import frc.robot.commands.InitializeLocalizationCommand;
import frc.robot.commands.RunAlgaeCollectorWheelsCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.subsystems.AlgaeArmSubsystem;

public class AutonScoreAndPickup_Blue0 extends SequentialCommandGroup {
    public AutonScoreAndPickup_Blue0(DriveSubsystem driveSubsystem,
            LocalizationSubsystem localizationSubsystem,
            VisionSubsystem visionSubsystem,
            NewCoralArmSubsystem coralArmSubsystem,
            AlgaeArmSubsystem algaeArmSubsystem) {
        addCommands(
                // 1. Initialize localization.
                new InitializeLocalizationCommand(driveSubsystem, localizationSubsystem),

            // 2. Drive to the scoring location with preloaded gamepiece while concurrently setting the arm.
            new ParallelCommandGroup(
                new DriveToPoseCommand(driveSubsystem, ReefscapeTargetPoses.BLUE_TAG22_LEFT, localizationSubsystem)
            ),
            //InitAlgaeCollectorPositionCommand
            // 3. Score the preloaded coral while driving to the pickup location.
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.1), // Delay 0.5 seconds before starting arm set.
                    new InitAlgaeCollectorPositionCommand(algaeArmSubsystem, 20),
                    new WaitCommand(0.1),
                    new RunAlgaeCollectorWheelsCommand(algaeArmSubsystem, 1.0,0.75),
                    new WaitCommand(0.1)
                )
            ),

                // 4. Back up from the reef to find a location to turn and drive to
                new ParallelCommandGroup(
                        new DriveToPoseCommand(driveSubsystem, ReefscapeTargetPoses.BLUE_0_INT, localizationSubsystem)),

                // 5. drive to coral station
                new ParallelCommandGroup(
                        new DriveToPoseCommand(driveSubsystem, ReefscapeTargetPoses.BLUE_CORAL_STATION_0,
                                localizationSubsystem)),

                // 6. Wait for coral to be deposited
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(2.0))),

                // 7. Drive back up to reef
                new ParallelCommandGroup(
                        new DriveToPoseCommand(driveSubsystem, ReefscapeTargetPoses.BLUE_TAG17_LEFT,
                                localizationSubsystem),
                        new SequentialCommandGroup(
                                new WaitCommand(0.1), // Delay 0.5 seconds before starting arm set.
                                new SetArmPositionCommand(coralArmSubsystem, ArmConstants.highTgtAngle,
                                        ArmConstants.highTgtHeight))),

                // 8. Score high
                new ParallelCommandGroup(
                        new WaitCommand(1.0),
                        new HighScoringSequenceCommand(coralArmSubsystem),
                        new WaitCommand(2.0)));
    }
}
