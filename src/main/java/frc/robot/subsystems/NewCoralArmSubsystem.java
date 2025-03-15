// NewCoralArmSubsystem.java
// Control Freaks 2025 – Revised coral arm subsystem with soft stops based on angle

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class NewCoralArmSubsystem extends SubsystemBase {
   private final SparkMax m_armExtend;
   private final SparkMax m_armAngle;
   private final RelativeEncoder e_armExtend;
   private final AbsoluteEncoder e_armAngle;
    
   // For PI control of coral extender and coral arm angle
   private double ce_integral;
   private double ca_integral;  

   // Need home positions for extender and arm angle
   private double ca_initTgt;
   private double ce_initTgt;

   private double ca_cmd;
   private double ca_error;


   private double ce_cmd;
   private double ce_error;
   private double currentAngle;

      // Limit math
      private double lim_m;
      private double lim_b;
      //private double maxAllowed;

   public NewCoralArmSubsystem() {
      m_armExtend = new SparkMax(ArmConstants.kCoralExtendCanId, MotorType.kBrushless);
      m_armAngle  = new SparkMax(ArmConstants.kCoralAngleExtenderCanId, MotorType.kBrushless);
        
      e_armExtend = m_armExtend.getEncoder();
      e_armAngle  = m_armAngle.getAbsoluteEncoder();
        
      // Reset encoders on startup (or before autonomous)
      zeroEncoders();

      ca_integral = 0.0;
      ce_integral = 0.0;
      lim_m = -2.25;
      lim_b = 140;
      ca_error = 0.0;
      ce_error = 0.0;
   }
    
   /******************************************************* 
    * Resets the encoders to zero and captures the values
    * for use in returning to the home position (INIT)
    * Call this before the autonomous run 
    * after manually positioning the arm at home.
    * 
    ******************************************************/
   public void zeroEncoders() {
      e_armExtend.setPosition(0.0);
      //e_armAngle.setPosition(0.0);

      ca_initTgt = 0.0;//e_armAngle.getPosition();
      ce_initTgt = e_armExtend.getPosition();
      currentAngle = 0.0;
   }

   public double getInitArmAngle() {
      return(ca_initTgt);
   }

   public double getInitArmExtend() {
      return(ce_initTgt);
   }
    
   /******************************************************
   * Closed-loop control for the arm angle using PI control.
   * @param targetAngle The desired encoder value for the arm angle.
   ******************************************************/
   public void setArmAngle(double targetAngle) {
      currentAngle = e_armAngle.getPosition();
      if (currentAngle > 0.9) currentAngle = 0.0;

      ca_error = (targetAngle - currentAngle);
      ca_integral += ca_error;

      // this prevents the I term for getting oversaturated
      // we really only want it to work to get remove the last 
      // little chunk of error
      if (ca_integral > ArmConstants.CA_I_MAX) ca_integral = ArmConstants.CA_I_MAX;
      if (ca_integral < -ArmConstants.CA_I_MAX) ca_integral = -ArmConstants.CA_I_MAX;

      ca_cmd = ca_error * ArmConstants.CA_PGain + ca_integral * ArmConstants.CA_IGain;

      // this saturates the command making sure it is never too large
      if (ca_cmd > ArmConstants.CA_MAX) ca_cmd = ArmConstants.CA_MAX;
      if (ca_cmd < -ArmConstants.CA_MAX) ca_cmd = -ArmConstants.CA_MAX;
   
      if (targetAngle < 0.01) {
         if (currentAngle < 0.05) {
            ca_cmd = ca_cmd * 0.5;
         }
         if (currentAngle < 0.01) {
            ca_cmd = ca_cmd *0.5;
         }
      }

      m_armAngle.set(ca_cmd);//ca_cmd);//command);
   }
    
   /*******************************************************
   * Closed-loop control for the arm extension using P control.
   * @param targetExtension The desired encoder value for extension.
   ******************************************************/
   public void moveArm(double targetExtension) {
      double currentExtension = e_armExtend.getPosition();
      ce_error = targetExtension - currentExtension;
      ce_integral += ce_error;

      // this prevents the I term for getting oversaturated
      // we really only want it to work to get remove the last 
      // little chunk of error
      if (ce_integral > ArmConstants.CE_I_MAX) ce_integral = ArmConstants.CE_I_MAX;
      if (ce_integral < -ArmConstants.CE_I_MAX) ce_integral = -ArmConstants.CE_I_MAX;

      double ce_cmd = ce_error * ArmConstants.CE_PGain + ce_integral * ArmConstants.CE_IGain;

      // this saturates the command making sure it is never too large
      if (ce_cmd > ArmConstants.CE_MAX) ce_cmd = ArmConstants.CE_MAX;
      if (ce_cmd < -ArmConstants.CE_MAX) ce_cmd = -ArmConstants.CE_MAX;

      m_armExtend.set(ce_cmd);
   }
    
   /*******************************************************
   * Manual open-loop adjustment for arm angle with soft stops.
   * If lowering the arm (speed < 0) would put the current extension out of its safe zone
   * for the new angle, then the command is clamped.
   * @param speed A value that directly commands the motor.
   ******************************************************/
   public void manualAdjustArmAngle(double speed) {
      currentAngle = e_armAngle.getPosition();
      if (currentAngle > 0.9) currentAngle = 0.0;
      double homeAngle = 0.0; // Define your home angle here (assumed 0)
      double newAngle = currentAngle + speed;

      // Check if the movement is toward home.
      // If the absolute difference from home is reduced, then allow the movement.
      if (Math.abs(newAngle - homeAngle) < Math.abs(currentAngle - homeAngle)) {
         m_armAngle.set(speed);
         return;
      }
      // Otherwise, enforce the soft-stop constraint based on extension.
      double currentExtension = e_armExtend.getPosition();
      double maxAllowedForNewAngle = getMaxAllowedExtension(newAngle);
    
      // For lowering the arm (speed < 0), if the current extension is more extended than allowed, block the command.
      if (speed < 0 && currentExtension < maxAllowedForNewAngle) {
         m_armAngle.set(0);
      } else {
         m_armAngle.set(speed);
      }
   }

   /*******************************************************
   * Manual adjustment for arm extension with soft stops that depend on the current angle.
   * Assumes that the extension encoder decreases (becomes more negative) as the arm extends.
   * @param speed A value that directly commands the motor.
   ******************************************************/
   public void manualAdjustArmExtension(double speed) {
      double currentExtension = e_armExtend.getPosition();
      currentAngle = e_armAngle.getPosition();
      if (currentAngle > 0.9) currentAngle = 0.0;
    
      // Compute the maximum allowed extension (a negative number) for the current angle.
      double maxAllowed = getMaxAllowedExtension(currentAngle);
    
      if (speed < 0) { // Operator is extending further.
         if (currentExtension <= maxAllowed) {
            // Already at (or beyond) the safe limit; block further extension.
            m_armExtend.set(0);
         } else {
            m_armExtend.set(speed);
         }
      } else if (speed > 0) { // Operator is retracting (moving toward 0, the home).
         // Allow retraction regardless of the soft-stop limit.
         m_armExtend.set(speed);
      } else {
         m_armExtend.set(0);
      }
   }

   /*******************************************************
   * Computes the maximum allowed extension for a given arm angle.
   * This method uses your preset target extension values (for low, mid, and high positions),
   * applies a 10% buffer.
   *
   * Adjust the interpolation as needed for your mechanism’s geometry.
   *
   * @param currentAngle The current encoder value for the arm angle.
   * @return The maximum safe extension (a negative value).
   ******************************************************/
   public double getMaxAllowedExtension(double currentAngle) {
      // alternate limit logic
      boolean useLinearLimitLogic = true;

      if (useLinearLimitLogic) {
         // angle = 40 > mx+b > -90 + 140 = 50
         // angle = 10 > mx+b > -25 + 140 = 115
         double max = currentAngle*lim_m + lim_b;
         return -max;
      } else {
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
   }

   /** Stops both motors. */
   public void stopArm() {
      m_armAngle.stopMotor();
      m_armExtend.stopMotor();
   }
    
   @Override
   public void periodic() {
      //double currentAngle = e_armAngle.getPosition();
      //if (currentAngle > 0.9) currentAngle = 0.0;
      //SmartDashboard.putNumber("Arm error", ca_error);
      SmartDashboard.putNumber("Arm Extension", e_armExtend.getPosition());
      SmartDashboard.putNumber("Arm Angle", e_armAngle.getPosition());
      //SmartDashboard.putNumber("CA Cmd", ca_cmd);
      // SmartDashboard.putNumber("Ex error", ce_error);
      // SmartDashboard.putNumber("Ex Cmd", ce_cmd);
      // SmartDashboard.putNumber("Ex Pos", e_armExtend.getPosition());
      //private double ;
      //private double ce_cmd;
   }
    
   public double getArmAngle() {
      return currentAngle;
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