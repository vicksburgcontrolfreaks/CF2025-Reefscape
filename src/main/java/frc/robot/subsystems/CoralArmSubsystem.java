// Control Freaks 2025
// controls the coral delivery subsystem

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.RelativeEncoder;

public class CoralArmSubsystem extends SubsystemBase {

   private final SparkMax m_armExtend;
   private final SparkMax m_armAngle;
   private RelativeEncoder e_armExtend;
   private RelativeEncoder e_armAngle;
   private double extendTgt;
   private double ce_initPos;
   private double ca_initPos;
   private double ca_errorPrev;
   private double ca_integral;
   double angleTgt = 0;

   public CoralArmSubsystem() {
      m_armExtend = new SparkMax(ArmConstants.kCoralExtendCanId, MotorType.kBrushless);
      m_armAngle = new SparkMax(ArmConstants.kCoralAngleExtenderCanId, MotorType.kBrushless);

      e_armExtend = m_armExtend.getEncoder();
      e_armAngle = m_armAngle.getEncoder();

      e_armExtend.setPosition(0.0);
      e_armAngle.setPosition(0.0);

      ce_initPos = e_armExtend.getPosition();
      ca_initPos = e_armAngle.getPosition();

      ca_integral = 0.0;
      ca_errorPrev = 0.0;
   }

   public void setArmAngle(int position) {
      double ca_cmd;
      double ca_error;

      // 0 - initial retracted position
      // 1 - lowest coral placement
      // 2 - mid coral placement
      // 3 - highest position

      if (position == ArmConstants.TGT_INIT)
         angleTgt = ca_initPos;
      if (position == ArmConstants.TGT_LOW)
         angleTgt = ca_initPos + ArmConstants.lowTgtAngle;
      if (position == ArmConstants.TGT_MID)
         angleTgt = ca_initPos + ArmConstants.midTgtAngle;
      if (position == ArmConstants.TGT_HIGH)
         angleTgt = ca_initPos + ArmConstants.highTgtAngle;

      ////// CORAL ROTATION ANGLE LOGIC using PI control - goal is to drive error to
      ////// zero
      ca_error = angleTgt - e_armAngle.getPosition(); // calculate the error between desired and current
      ca_integral = ca_integral + ca_errorPrev; // this is the error added up over time
      ca_cmd = ca_error * ArmConstants.CA_PGain + ca_integral * ArmConstants.CA_IGain; // both P and I terms combined

      // This logic just saturates the command making sure it doesn't get too strong
      if (ca_cmd > ArmConstants.CA_MAX) {
         ca_cmd = ArmConstants.CA_MAX;
      }
      if (ca_cmd < -ArmConstants.CA_MAX) {
         ca_cmd = -ArmConstants.CA_MAX;
      }

      m_armAngle.set(ca_cmd);
      ca_errorPrev = ca_error;
   }

   public void moveArm(int position) {
      double ce_cmd;
      double ce_error;
      // 0 - initial retracted position
      // 1 - lowest coral placement
      // 2 - mid coral placement
      // 3 - highest position

      if (position == ArmConstants.TGT_INIT)
         extendTgt = ce_initPos;
      if (position == ArmConstants.TGT_LOW)
         extendTgt = ce_initPos + ArmConstants.lowTgtHeight;
      if (position == ArmConstants.TGT_MID)
         extendTgt = ce_initPos + ArmConstants.midTgtHeight;
      if (position == ArmConstants.TGT_HIGH)
         extendTgt = ce_initPos + ArmConstants.highTgtHeight;

      ////// CORAL EXTENSION LENGTH uses only P control - goal is to drive error to
      ////// zero
      ce_error = extendTgt - e_armExtend.getPosition(); // calculate the error between desired and current
      ce_cmd = ArmConstants.CE_PGain * ce_error; // error times P gain gives command

      // This logic just saturates the command making sure it doesn't get too strong
      if (ce_cmd > ArmConstants.CE_MAX) {
         ce_cmd = ArmConstants.CE_MAX;
      }
      if (ce_cmd < -ArmConstants.CE_MAX) {
         ce_cmd = -ArmConstants.CE_MAX;
      }

      m_armExtend.set(ce_cmd);
   }

   public void stopArm() {
      m_armExtend.stopMotor();
      m_armAngle.stopMotor();
   }

   @Override
   public void periodic() {
      // Publish current arm angle and extension positions to SmartDashboard.
      SmartDashboard.putNumber("Coral Arm Angle", e_armAngle.getPosition());
      SmartDashboard.putNumber("Coral Arm Extension", e_armExtend.getPosition());
      SmartDashboard.putNumber("Coral An Tgt", angleTgt);
      SmartDashboard.putNumber("Coral Ex Tgt", extendTgt);
   }

   /*
    * Manually adjust the arm angle by directly setting the motor output.
    * 
    * @param speed The speed (voltage output) for the angle motor.
    */
   public void manualAdjustArmAngle(double speed) {
      m_armAngle.set(speed);
   }

   /**
    * Manually adjust the arm extension by directly setting the motor output.
    * 
    * @param speed The speed (voltage output) for the extension motor.
    */
   public void manualAdjustArmExtension(double speed) {
      m_armExtend.set(speed);
   }

   // Returns the current drawn by the angle motor.
   public double getAngleCurrent() {
      return m_armAngle.getOutputCurrent(); // Using SparkMax API (adjust if needed)
   }

   // Returns the current drawn by the extension motor.
   public double getExtendCurrent() {
      return m_armExtend.getOutputCurrent(); // Using SparkMax API (adjust if needed)
   }

   // Returns the current extension position (encoder value).
   public double getCurrentExtension() {
      return e_armExtend.getPosition();
   }

   /**
    * Sets the arm angle and extension to the specified target positions using
    * closed-loop control.
    *
    * @param targetAngle     The desired arm angle (in encoder units or your chosen
    *                        unit).
    * @param targetExtension The desired arm extension (in encoder units).
    * @param speedFactor     A scaling factor (0.0 to 1.0) to adjust the motor
    *                        outputs.
    */
   public void setArmPosition(double targetAngle, double targetExtension, double speedFactor) {
      // --- Angle Control (using PI control) ---
      double currentAngle = e_armAngle.getPosition();
      double errorAngle = targetAngle - currentAngle;

      // Update integral (using the current error)
      ca_integral += errorAngle;

      // Compute the command for the angle motor using PI control
      double angleCmd = errorAngle * ArmConstants.CA_PGain + ca_integral * ArmConstants.CA_IGain;

      // Scale the command by the speed factor
      angleCmd *= speedFactor;

      // Saturate the command to ensure it doesn't exceed the maximum output
      if (angleCmd > ArmConstants.CA_MAX) {
         angleCmd = ArmConstants.CA_MAX;
      } else if (angleCmd < -ArmConstants.CA_MAX) {
         angleCmd = -ArmConstants.CA_MAX;
      }

      // Set the angle motor output
      m_armAngle.set(angleCmd);

      // Save the current error for any additional integration logic (if needed)
      ca_errorPrev = errorAngle;

      // --- Extension Control (using P control) ---
      double currentExtension = e_armExtend.getPosition();
      double errorExtension = targetExtension - currentExtension;

      // Compute the command for the extension motor using P control
      double extendCmd = errorExtension * ArmConstants.CE_PGain;

      // Scale the command by the speed factor
      extendCmd *= speedFactor;

      // Saturate the command to ensure it doesn't exceed the maximum output
      if (extendCmd > ArmConstants.CE_MAX) {
         extendCmd = ArmConstants.CE_MAX;
      } else if (extendCmd < -ArmConstants.CE_MAX) {
         extendCmd = -ArmConstants.CE_MAX;
      }

      // Set the extension motor output
      m_armExtend.set(extendCmd);
   }

   public double getArmAngle() {
      return e_armAngle.getPosition();
   }

}
