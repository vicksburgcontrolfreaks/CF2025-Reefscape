// File: HarpoonSubsystem.java
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HarpoonSubsystem extends SubsystemBase {
    private final SparkMax m_harpoon;

    public HarpoonSubsystem() {
        m_harpoon = new SparkMax(ArmConstants.HarpoonCanId, MotorType.kBrushless);
    }

    /**
     * Sets the motor speed for the harpoon.
     * @param speed The motor output (range: -1.0 to 1.0).
     */
    public void setMotor(double speed) {
        m_harpoon.set(speed);
    }

    /**
     * Stops the harpoon motor.
     */
    public void stop() {
        m_harpoon.stopMotor();
    }

    @Override
    public void periodic() {
        // Display current motor output on SmartDashboard for debugging.
        // SmartDashboard.putNumber("Harpoon Output", m_harpoon.get());
    }
}
