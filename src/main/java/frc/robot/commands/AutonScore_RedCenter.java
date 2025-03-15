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

public class AutonScore_RedCenter extends SequentialCommandGroup {
    public AutonScore_RedCenter(DriveSubsystem driveSubsystem,
                                   LocalizationSubsystem localizationSubsystem,
                                   VisionSubsystem visionSubsystem,
                                   NewCoralArmSubsystem coralArmSubsystem,
                                   AlgaeArmSubsystem algaeArmSubsystem) {
        addCommands(
            // 1. Initialize localization.
            new InitializeLocalizationCommand(driveSubsystem, localizationSubsystem),

            // 2. Drive to the scoring location with preloaded gamepiece while concurrently setting the arm.
            new ParallelCommandGroup(
                new DriveToPoseCommand(driveSubsystem, ReefscapeTargetPoses.RED_TAG10_RIGHT, localizationSubsystem)
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
            )
        );
    }
}
