// File: CoralCollectorSubsystem.java
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralCollectorSubsystem extends SubsystemBase {
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final double tolerance = 0.5;
    double error;
    double output;

    public CoralCollectorSubsystem() {
        // Use the CAN ID from your constants.
        m_motor = new SparkMax(Constants.ArmConstants.CoralCollectorCanId, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        m_encoder.setPosition(0.0);
    }

    /**
     * Drives the motor using a simple proportional controller toward the target
     * position.
     * 
     * @param target The desired encoder position.
     */

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void setPosition(double target) {
        double current = m_encoder.getPosition();
        error = target - current;
        // Example proportional gain; adjust as needed.
        output = error * 0.4;
        // Clamp the output to a safe range.
        output = Math.max(-0.5, Math.min(0.5, output));
        m_motor.set(output);
    }

    /**
     * Stops the motor.
     */
    public void stop() {
        m_motor.stopMotor();
    }

    public void zeroEncoders() {
        m_encoder.setPosition(0.0);

    }

    @Override
    public void periodic() {
        if (Constants.COMP_CODE){
        // Not really interested in seeing anything for this one.
        }
        else{
        // Publish the current encoder position for debugging.
        SmartDashboard.putNumber("CoralCollector Position", m_encoder.getPosition());
        SmartDashboard.putNumber("CoralCollector Error", error);
        SmartDashboard.putNumber("CoralCollector Output", output);
        }
    }
}
