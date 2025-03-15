// File: InitializeLocalizationCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class InitializeLocalizationCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final LocalizationSubsystem localizationSubsystem;

    public InitializeLocalizationCommand(DriveSubsystem driveSubsystem, LocalizationSubsystem localizationSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.localizationSubsystem = localizationSubsystem;
        // Require both subsystems to ensure no other command interferes.
        addRequirements(driveSubsystem, localizationSubsystem);
    }

    @Override
    public void initialize() {
        // Attempt to get a vision-based pose estimate.
        var visionEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (visionEstimate != null && visionEstimate.tagCount > 0) {
            // Reset odometry to the vision measurement.
            driveSubsystem.resetOdometry(visionEstimate.pose);
            // SmartDashboard.putString("Localization Init", "Odometry reset using vision");
        } else {
            // SmartDashboard.putString("Localization Init", "No valid vision measurement");
        }
    }

    @Override
    public boolean isFinished() {
        // Ends immediately.
        return true;
    }
}
