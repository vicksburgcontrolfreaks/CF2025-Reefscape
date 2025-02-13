// File: CurrentMonitor.java
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.MAXSwerveModule;

public class CurrentMonitor {
    // Member variables for maximum current values.
    private double m_frontLeftMaxCurrent = 0.0;
    private double m_frontRightMaxCurrent = 0.0;
    private double m_rearLeftMaxCurrent = 0.0;
    private double m_rearRightMaxCurrent = 0.0;
    
    // References to the swerve modules.
    private final MAXSwerveModule m_frontLeft;
    private final MAXSwerveModule m_frontRight;
    private final MAXSwerveModule m_rearLeft;
    private final MAXSwerveModule m_rearRight;
    
    /**
     * Constructs a CurrentMonitor with references to each MAXSwerveModule.
     *
     * @param frontLeft  The front-left module.
     * @param frontRight The front-right module.
     * @param rearLeft   The rear-left module.
     * @param rearRight  The rear-right module.
     */
    public CurrentMonitor(MAXSwerveModule frontLeft, MAXSwerveModule frontRight,
                          MAXSwerveModule rearLeft, MAXSwerveModule rearRight) {
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_rearLeft = rearLeft;
        m_rearRight = rearRight;
    }
    
    /**
     * Call this method periodically (e.g., from your DriveSubsystem.periodic())
     * to update and log the live and maximum current values.
     */
    public void update() {
        // Retrieve the current draw for each module.
        double frontLeftCurrent = m_frontLeft.getCurrent();
        double frontRightCurrent = m_frontRight.getCurrent();
        double rearLeftCurrent = m_rearLeft.getCurrent();
        double rearRightCurrent = m_rearRight.getCurrent();
        
        // Update the maximum current values.
        m_frontLeftMaxCurrent = Math.max(m_frontLeftMaxCurrent, frontLeftCurrent);
        m_frontRightMaxCurrent = Math.max(m_frontRightMaxCurrent, frontRightCurrent);
        m_rearLeftMaxCurrent = Math.max(m_rearLeftMaxCurrent, rearLeftCurrent);
        m_rearRightMaxCurrent = Math.max(m_rearRightMaxCurrent, rearRightCurrent);
        
        // Publish live current values.
        SmartDashboard.putNumber("Front Left Current", frontLeftCurrent);
        SmartDashboard.putNumber("Front Right Current", frontRightCurrent);
        SmartDashboard.putNumber("Rear Left Current", rearLeftCurrent);
        SmartDashboard.putNumber("Rear Right Current", rearRightCurrent);
        
        // Publish maximum current values.
        SmartDashboard.putNumber("Front Left Max Current", m_frontLeftMaxCurrent);
        SmartDashboard.putNumber("Front Right Max Current", m_frontRightMaxCurrent);
        SmartDashboard.putNumber("Rear Left Max Current", m_rearLeftMaxCurrent);
        SmartDashboard.putNumber("Rear Right Max Current", m_rearRightMaxCurrent);
    }
    
    /**
     * Resets the stored maximum current values.
     */
    public void resetMaxCurrents() {
        m_frontLeftMaxCurrent = 0.0;
        m_frontRightMaxCurrent = 0.0;
        m_rearLeftMaxCurrent = 0.0;
        m_rearRightMaxCurrent = 0.0;
    }
}
