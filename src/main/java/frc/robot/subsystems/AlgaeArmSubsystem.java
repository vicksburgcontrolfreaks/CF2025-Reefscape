// File: AlgaeArmSubsystem.java
package frc.robot.subsystems;

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

    // State variables.
    private double wheelCounter;
    private boolean collectionComplete; // Indicates ball collection complete.
    private double holdPosition;        // The arm position to hold once ball is collected.

    // Placeholder target for arm extension (encoder units).
    private final double armExtendTarget = 50; 
    // Current threshold for wheel current (in Amps).
    private final double wheelStopSoftCap = 30.0; // Adjust based on testing.

    // Proportional gain for holding the arm position.
    private final double kHoldP = 0.1;  // Tune this value as needed.

    public AlgaeArmSubsystem() {
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
     * Runs the collector action.
     * Extends the arm until it reaches armExtendTarget and runs the wheel motor to collect the ball.
     * For the first 10 cycles, the current is not evaluated to allow startup current to settle.
     * Once the wheel motor current exceeds the threshold, the system records the current arm position,
     * stops the wheels, and holds the arm at that position using a simple proportional control.
     *
     * @return true when ball collection is complete and the arm is held.
     */
    public boolean algaeArmCollect() {
        // If collection is not complete, run collection actions.
        if (!collectionComplete) {
            // Extend the arm until reaching the target.
            if (armEncoder.getPosition() < armExtendTarget) {
                armMotor.set(0.5); // Extend at 50% power.
            } else {
                armMotor.set(0);
            }
            
            // Run the collector wheels.
            wheelMotor.set(0.20);
  
            // Delay evaluation of the wheel current for the first 10 cycles.
            if (wheelCounter < 10) {
                wheelCounter++;
            } else {
                // After delay, if current exceeds threshold, mark collection complete.
                if (wheelMotor.getOutputCurrent() >= wheelStopSoftCap) {
                    collectionComplete = true;
                    // Capture current arm position to hold.
                    holdPosition = armEncoder.getPosition();
                }
            }
        } else {
            // Ball is collected; hold the arm at the captured position.
            double error = holdPosition - armEncoder.getPosition();
            double output = error * kHoldP;
            // Clamp the output if necessary.
            output = Math.max(-1.0, Math.min(1.0, output));
            armMotor.set(output);
            // Ensure wheels are stopped.
            wheelMotor.set(0);
        }
        return collectionComplete;
    }

    /**
     * Runs the release action: reverses the wheel motor to eject the ball and retracts the arm until
     * the encoder reaches zero.
     *
     * @return true when the release and retraction are complete.
     */
    public boolean algaeArmShoot() {
        // For simplicity, run the wheel motor in reverse for 10 iterations then stop.
        if (wheelCounter < 10) {
            wheelMotor.set(-1.0);
            wheelCounter++;
        } else {
            wheelMotor.set(0);
        }
        
        // Retract the arm continuously until it is at or below zero.
        if (armEncoder.getPosition() > 0) {
            armMotor.set(-0.5);
            return false;
        } else {
            armMotor.set(0);
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
