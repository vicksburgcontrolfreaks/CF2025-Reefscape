// NewCoralArmSubsystem.java
// Control Freaks 2025 – Revised coral arm subsystem with soft stops based on angle

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class NewCoralArmSubsystem extends SubsystemBase {
    private final SparkMax m_armExtend;
    private final SparkMax m_armAngle;
    private final RelativeEncoder e_armExtend;
    private final RelativeEncoder e_armAngle;
    
    // For PI control on the angle motor.
    private double ca_integral;
    private double ca_errorPrev;
    
    public NewCoralArmSubsystem() {
        m_armExtend = new SparkMax(ArmConstants.kCoralExtendCanId, MotorType.kBrushless);
        m_armAngle  = new SparkMax(ArmConstants.kCoralAngleExtenderCanId, MotorType.kBrushless);
        
        e_armExtend = m_armExtend.getEncoder();
        e_armAngle  = m_armAngle.getEncoder();
        
        // Reset encoders on startup (or before autonomous)
        zeroEncoders();
        
        ca_integral = 0.0;
        ca_errorPrev = 0.0;
    }
    
    /** 
     * Resets the encoders to zero. Call this before the autonomous run 
     * after manually positioning the arm at home.
     */
    public void zeroEncoders() {
        e_armExtend.setPosition(0.0);
        e_armAngle.setPosition(0.0);
    }
    
    /**
     * Closed-loop control for the arm angle using PI control.
     * @param targetAngle The desired encoder value for the arm angle.
     */
    public void setArmAngle(double targetAngle) {
        double currentAngle = e_armAngle.getPosition();
        double error = targetAngle - currentAngle;
        ca_integral += error;
        double command = error * ArmConstants.CA_PGain + ca_integral * ArmConstants.CA_IGain;
        if (command > ArmConstants.CA_MAX) command = ArmConstants.CA_MAX;
        if (command < -ArmConstants.CA_MAX) command = -ArmConstants.CA_MAX;
        m_armAngle.set(command);
        ca_errorPrev = error;
    }
    
    /**
     * Closed-loop control for the arm extension using P control.
     * @param targetExtension The desired encoder value for extension.
     */
    public void moveArm(double targetExtension) {
        double currentExtension = e_armExtend.getPosition();
        double error = targetExtension - currentExtension;
        double command = error * ArmConstants.CE_PGain;
        if (command > ArmConstants.CE_MAX) command = ArmConstants.CE_MAX;
        if (command < -ArmConstants.CE_MAX) command = -ArmConstants.CE_MAX;
        m_armExtend.set(command);
    }
    
  /**
 * Manual open-loop adjustment for arm angle with soft stops.
 * If lowering the arm (speed < 0) would put the current extension out of its safe zone
 * for the new angle, then the command is clamped.
 * @param speed A value that directly commands the motor.
 */
public void manualAdjustArmAngle(double speed) {
    double currentAngle = e_armAngle.getPosition();
    double currentExtension = e_armExtend.getPosition();
    
    // Predict a new angle based on current angle and the commanded speed.
    // (This is an open-loop prediction; in a real system you might consider a time delta.)
    double newAngle = currentAngle + speed;
    
    // Compute the maximum allowed (i.e. most negative) extension for the predicted new angle.
    double maxAllowedForNewAngle = getMaxAllowedExtension(newAngle);
    
    // If lowering the arm (speed < 0) would leave the arm extended more than allowed,
    // then cancel further lowering.
    if (speed < 0 && currentExtension < maxAllowedForNewAngle) {
        m_armAngle.set(0);
    } else {
        m_armAngle.set(speed);
    }
}

/**
 * Manual adjustment for arm extension with soft stops that depend on the current angle.
 * Assumes that the extension encoder decreases (becomes more negative) as the arm extends.
 * @param speed A value that directly commands the motor.
 */
public void manualAdjustArmExtension(double speed) {
    double currentExtension = e_armExtend.getPosition();
    double currentAngle = e_armAngle.getPosition();
    
    // Compute the maximum allowed extension at this angle.
    // Note: Since the encoder decreases from zero when extended, the safe limit is a negative number.
    double maxAllowed = getMaxAllowedExtension(currentAngle);
    
    // For extending further: speed < 0 makes the encoder more negative.
    // If already at or beyond (i.e. less than or equal to) the safe limit, stop extension.
    if (speed < 0 && currentExtension <= maxAllowed) {
        m_armExtend.set(0);
    }
    // For retracting: speed > 0 brings the encoder toward zero.
    // If already at or above zero, stop retraction.
    else if (speed > 0 && currentExtension >= 0) {
        m_armExtend.set(0);
    } else {
        m_armExtend.set(speed);
    }
}

/**
 * Computes the maximum allowed extension for a given arm angle.
 * This method uses your preset target extension values (for low, mid, and high positions),
 * applies a 10% buffer.
 *
 * Adjust the interpolation as needed for your mechanism’s geometry.
 *
 * @param currentAngle The current encoder value for the arm angle.
 * @return The maximum safe extension (a negative value).
 */
public double getMaxAllowedExtension(double currentAngle) {
    // Convert preset extension heights to limits with a 10% buffer.
    double lowLimit  = ArmConstants.lowTgtHeight  * 1.1;
    double midLimit  = ArmConstants.midTgtHeight  * 1.1;
    double highLimit = ArmConstants.highTgtHeight * 1.1;
    
    if (currentAngle <= ArmConstants.lowTgtAngle) {
        return lowLimit;
    } else if (currentAngle <= ArmConstants.midTgtAngle) {
        double t = (currentAngle - ArmConstants.lowTgtAngle) / (ArmConstants.midTgtAngle - ArmConstants.lowTgtAngle);
        return lowLimit + t * (midLimit - lowLimit);
    } else if (currentAngle <= ArmConstants.highTgtAngle) {
        double t = (currentAngle - ArmConstants.midTgtAngle) / (ArmConstants.highTgtAngle - ArmConstants.midTgtAngle);
        return midLimit + t * (highLimit - midLimit);
    } else {
        return highLimit;
    }
}

    
    /** Stops both motors. */
    public void stopArm() {
        m_armAngle.stopMotor();
        m_armExtend.stopMotor();
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", e_armAngle.getPosition());
        SmartDashboard.putNumber("Arm Extension", e_armExtend.getPosition());
    }
    
    public double getArmAngle() {
        return e_armAngle.getPosition();
    }
    
    public double getCurrentExtension() {
        return e_armExtend.getPosition();
    }

    public double getAngleCurrent() {
        return m_armAngle.getOutputCurrent();
    }
    
    public double getExtendCurrent() {
        return m_armExtend.getOutputCurrent();
    }
}
