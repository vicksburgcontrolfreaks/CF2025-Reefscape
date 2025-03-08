// File: AlgaeArmSubsystem.java
package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeArmSubsystem extends SubsystemBase {
    // Motor for extending/retracting the arm.
    private final SparkMax armMotor = new SparkMax(ArmConstants.AlgaeExtenderCanId, MotorType.kBrushless);
    // Motor for running the collector wheels.
    private final SparkMax wheelMotor = new SparkMax(ArmConstants.AlgaeCollectorCanId, MotorType.kBrushless);

    // Encoder for the arm.
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    // (Optionally, you can use the wheel encoder if needed.)
    private final RelativeEncoder wheelEncoder = wheelMotor.getEncoder();

    // State variables.
    private double wheelCounter;
    private boolean collectionComplete; // True when a ball is collected.
    private double holdPosition;        // The arm encoder position to hold after collection.

    // Current threshold for the wheel motor (in Amps).
    private final double wheelStopSoftCap = 30.0; // Adjust as needed.

    // Proportional gain for holding/positioning the arm.
    private final double kHoldP = 0.1;  // Tune this value as needed.

    public AlgaeArmSubsystem() {
        // Assume the arm is retracted at startup.
        armEncoder.setPosition(0.0);
        resetState();
        SmartDashboard.putNumber("AlgaeArm Encoder", armEncoder.getPosition());
    }

    /** Resets state variables for a new collection cycle. */
    public void resetState() {
        wheelCounter = 0;
        collectionComplete = false;
        holdPosition = 0;
    }

    /**
     * Resets both the arm and wheel encoders.
     */
    public void zeroEncoders() {
        armEncoder.setPosition(0.0);
        // If desired, reset the wheel encoder as well:
        wheelEncoder.setPosition(0.0);
    }

    /**
     * Sets the arm position using a simple proportional control.
     * This method calculates the error between the target position and the current encoder reading,
     * then sets the motor output proportional to that error.
     * @param targetPosition The desired encoder position.
     */
    public void setArmPosition(double targetPosition) {
        double error = targetPosition - armEncoder.getPosition();
        double output = error * kHoldP;
        // Clamp the output between -1 and 1.
        double clamp = 0.15;
        output = Math.max(-clamp, Math.min(clamp, output));
        armMotor.set(output);
    }

    /**
     * Directly sets the wheel motor output.
     * @param power Motor output (typically between -1 and 1).
     */
    public void setWheelMotor(double power) {
        wheelMotor.set(power);
    }

    /**
     * Collector action: extends the arm until reaching a target, then runs the wheels until a current spike
     * is detected, then holds the arm at that position.
     * @return true when ball collection is complete and the arm is held.
     */
    public boolean algaeArmCollect() {
        if (!collectionComplete) {
            // Extend the arm until the target extension is reached.
            if (armEncoder.getPosition() < Constants.AlgaeConstants.extended) {
                armMotor.set(0.5);
            } else {
                armMotor.set(0.0);
            }
            // Run the collector wheels.
            wheelMotor.set(0.20);
  
            // Allow a brief delay before evaluating wheel current.
            if (wheelCounter < 10) {
                wheelCounter++;
            } else {
                if (wheelMotor.getOutputCurrent() >= wheelStopSoftCap) {
                    collectionComplete = true;
                    // Record the current arm position so that it can be held.
                    holdPosition = armEncoder.getPosition();
                }
            }
        } else {
            // Ball collected; hold the arm at the recorded position.
            setArmPosition(holdPosition);
            // Ensure the wheels are stopped.
            wheelMotor.set(0.0);
        }
        return collectionComplete;
    }

    /**
     * Release action: reverses the wheel motor to eject the ball and retracts the arm until it reaches zero.
     * @return true when release and retraction are complete.
     */
    public boolean algaeArmShoot() {
        if (wheelCounter < 30) {
            wheelMotor.set(-1.0);
            wheelCounter++;
        } else {
            wheelMotor.set(0.0);
        }
        
        if (armEncoder.getPosition() > 0) {
            armMotor.set(-0.5);
            return false;
        } else {
            armMotor.set(0.0);
            return true;
        }
    }


    
    /** Stops both motors. */
    public void stop() {
        armMotor.stopMotor();
        wheelMotor.stopMotor();
    }

    /** Resets the arm encoder. */
    public void zeroArmEncoder() {
        armEncoder.setPosition(0.0);
    }

    /** Returns the current arm encoder position. */
    public double getArmPosition() {
        return armEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AlgaeArm Encoder", armEncoder.getPosition());
        SmartDashboard.putNumber("Wheel Counter", wheelCounter);
        SmartDashboard.putBoolean("Collection Complete", collectionComplete);
        SmartDashboard.putNumber("Wheel Current", wheelMotor.getOutputCurrent());
    }
}
