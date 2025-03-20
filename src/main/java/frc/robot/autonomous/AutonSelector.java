package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.NewCoralArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutonSelector {
    private final DriveSubsystem driveSubsystem;
    private final LocalizationSubsystem localizationSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final NewCoralArmSubsystem coralArmSubsystem;
    private final AlgaeArmSubsystem algaeArmSubsystem;

    /**
     * Constructs an AutoSelector.
     *
     * @param driveSubsystem         The drive subsystem.
     * @param localizationSubsystem  The localization subsystem.
     * @param visionSubsystem        The vision subsystem.
     * @param coralArmSubsystem      The coral arm subsystem.
     * @param algaeArmSubsystem      The algae arm subsystem.
     */
    public AutonSelector(DriveSubsystem driveSubsystem,
                        LocalizationSubsystem localizationSubsystem,
                        VisionSubsystem visionSubsystem,
                        NewCoralArmSubsystem coralArmSubsystem,
                        AlgaeArmSubsystem algaeArmSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.localizationSubsystem = localizationSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.coralArmSubsystem = coralArmSubsystem;
        this.algaeArmSubsystem = algaeArmSubsystem;
    }

    /**
     * Selects the autonomous command based on the current estimated pose.
     *
     * @return The autonomous Command to run.
     */
    public Command selectAutoCommand() {
        // Retrieve the current estimated pose.
        Pose2d pose = localizationSubsystem.getEstimatedPose();
        double x = pose.getTranslation().getX();
        double y = pose.getTranslation().getY();
        
        // Publish pose data for debugging.
        SmartDashboard.putNumber("Auto X", x);
        SmartDashboard.putNumber("Auto Y", y);
    
        Command autoCommand;
        
        // Logic for selecting a command based on field position.
        if (x > 8.8) { // Red side
            if (y > 5.0) {
                autoCommand = new AutonScoreAndPickup_Red1(driveSubsystem, localizationSubsystem, visionSubsystem,
                        coralArmSubsystem, algaeArmSubsystem);
            } else if (y < 3.0) {
                autoCommand = new AutonScoreAndPickup_Red0(driveSubsystem, localizationSubsystem, visionSubsystem,
                        coralArmSubsystem, algaeArmSubsystem);
            } else {
                autoCommand = new AutonScore_RedCenter(driveSubsystem, localizationSubsystem, visionSubsystem,
                        coralArmSubsystem, algaeArmSubsystem);
            }
        } else { // Blue side
            if (y > 5.0) {
                autoCommand = new AutonScoreAndPickup_Blue1(driveSubsystem, localizationSubsystem, visionSubsystem,
                        coralArmSubsystem, algaeArmSubsystem);
            } else if (y < 3.0) {
                autoCommand = new AutonScoreAndPickup_Blue0(driveSubsystem, localizationSubsystem, visionSubsystem,
                        coralArmSubsystem, algaeArmSubsystem);
            } else {
                autoCommand = new AutonScore_BlueCenter(driveSubsystem, localizationSubsystem, visionSubsystem,
                        coralArmSubsystem, algaeArmSubsystem);
            }
        }
        
        // Put the name of the selected autonomous command on SmartDashboard.
        SmartDashboard.putString("Auto Selected", autoCommand.getName());
        return autoCommand;
    }
    
}
