package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelemetrySubsystem extends SubsystemBase {

    private VisionSubsystem Vision_Subsystem;
    private SendableChooser<Command> Auto_Chooser;

    public TelemetrySubsystem(VisionSubsystem visionsubsystem, SendableChooser<Command> autoChooser) {
        this.Vision_Subsystem = visionsubsystem;
        this.Auto_Chooser = autoChooser;
    }

    public void ConfigureDashboard() {

        // Publish the VisionSubsystem to SmartDashboard for monitoring.
        SmartDashboard.putData("Vision Subsystem", Vision_Subsystem);
        // for choosing autonomous mode
        SmartDashboard.putData("Auto Tuning Mode", Auto_Chooser);
    }
}