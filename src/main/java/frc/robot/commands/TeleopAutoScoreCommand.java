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
    /**
     * @param driveSubsystem the drive subsystem
     * @param localizationSubsystem the localization subsystem
     * @param visionSubsystem the vision subsystem
     * @param coralArmSubsystem the coral arm subsystem
     * @param isLeft if true, drive toward the left target; if false, drive toward the right target.
     * @param targetArmPosition the desired arm position (LOW, MID, HIGH)
     */
    public TeleopAutoScoreCommand(DriveSubsystem driveSubsystem,
                                  LocalizationSubsystem localizationSubsystem,
                                  VisionSubsystem visionSubsystem,
                                  NewCoralArmSubsystem coralArmSubsystem,
                                  boolean isLeft) {
        // Determine the target arm angle and extension based on the argument.
        double targetAngle;
        double targetExtension;
        switch (ArmConstants.targetArmPosition) {
            case LOW:
                targetAngle = ArmConstants.lowTgtAngle;
                
                targetExtension = ArmConstants.lowTgtHeight;
                break;
            case MID:
                targetAngle = ArmConstants.midTgtAngle;
                targetExtension = ArmConstants.midTgtHeight;
                break;
            case HIGH:
                targetAngle = ArmConstants.highTgtAngle;
                targetExtension = ArmConstants.highTgtHeight;
                break;
            default:
                // Fallback to HIGH if not specified.
                targetAngle = ArmConstants.highTgtAngle;
                targetExtension = ArmConstants.highTgtHeight;
                break;
        }
        
        addCommands(
            // 1. Initialize localization.
            new InitializeLocalizationCommand(driveSubsystem, localizationSubsystem),
            
            // 2. Drive to the scoring location while concurrently setting the arm.
            new ParallelCommandGroup(
                new DynamicDriveToTagCommand(driveSubsystem, localizationSubsystem, isLeft),
                new SequentialCommandGroup(
                    new WaitCommand(0.5), // Delay before starting the arm command.
                    new SetArmPositionCommand(coralArmSubsystem, targetAngle, targetExtension)
                )
            )
        );
        if (ArmConstants.currentArmPosition != ArmConstants.targetArmPosition)
        {
            ArmConstants.currentArmPosition = ArmConstants.targetArmPosition;
        }
    }
}
