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
    private boolean springStartup;
    private boolean end;

    // Placeholder target for arm extension.
    private final double armExtendTarget = 50; // in encoder units
    // Current threshold for wheel current (in Amps).
    private final double wheelStopSoftCap = 30.0; // adjust based on testing

    public AlgaeArmSubsystem() {
        armEncoder.setPosition(0.0);
        resetState();
        SmartDashboard.putNumber("AlgaeArm Encoder", armEncoder.getPosition());
    }

    /** Resets state variables for a new command run. */
    public void resetState() {
        wheelCounter = 0;
        springStartup = false;
        end = false;
    }

    /**
     * Runs the collector action.
     * Extends the arm until it reaches armExtendTarget and runs the wheel motor
     * until a current spike indicates a ball is collected.
     *
     * @return true when ball collection is complete.
     */
    public boolean algaeArmCollect() {
      // Extend the arm until it reaches the target position.
      if (armEncoder.getPosition() < armExtendTarget) {
          armMotor.set(0.5); // Extend at 50% speed.
      } else {
          armMotor.set(0);   // Stop extending once target is reached.
      }
      
      // Run the wheel motor at full power to collect the ball.
      wheelMotor.set(0.20);
  
      // Delay evaluating the wheel current for the first 10 cycles.
      if (wheelCounter < 10) {
          wheelCounter++;
      } else {
          // After the delay, check if the current exceeds the threshold.
          if (wheelMotor.getOutputCurrent() >= wheelStopSoftCap) {
              end = true;
          }
      }
      
      return end;
  }
  

    /**
     * Runs the release action: reverses the wheel motor to eject the ball and
     * retracts the arm until the encoder reads zero.
     *
     * @return true when release is complete.
     */
    public boolean algaeArmShoot() {
      // Run the wheel motor in reverse for a fixed number of iterations.
      if (wheelCounter < 10) {
          wheelMotor.set(-1.0);
          wheelCounter++;
      } else {
          wheelMotor.set(0);
      }
      
      // Retract the arm continuously until the encoder indicates the arm is at zero.
      if (armEncoder.getPosition() > 0) {
          armMotor.set(-0.5);
          return false; // Not finished until the arm is fully retracted.
      } else {
          armMotor.set(0);
          return true; // Finished when the arm is at or below zero.
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
        SmartDashboard.putBoolean("Collection Complete", end);
        SmartDashboard.putNumber("Wheel Current", wheelMotor.getOutputCurrent());
    }
}
