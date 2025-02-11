package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class TelemetrySubsystem extends SubsystemBase {

    public TelemetrySubsystem() {}

    public void ConfigureDashboard() {

        // Publish the VisionSubsystem to SmartDashboard for monitoring.
        SmartDashboard.putData("Vision Subsystem", Robot.m_robotContainer.m_visionSubsystem);
        // for choosing autonomous mode
        SmartDashboard.putData("Auto Tuning Mode", Robot.m_robotContainer.autoChooser);
    }
}