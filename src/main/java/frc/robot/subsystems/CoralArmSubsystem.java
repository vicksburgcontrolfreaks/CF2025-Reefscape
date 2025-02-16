// Control Freaks 2025
// controls the coral delivery subsystem

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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

   public CoralArmSubsystem() {
      m_armExtend = new SparkMax(ArmConstants.kCoralExtendCanId, MotorType.kBrushless);
      m_armAngle  = new SparkMax(ArmConstants.kCoralAngleExtenderCanId,MotorType.kBrushless);
      
      e_armExtend = m_armExtend.getEncoder();
      e_armAngle  = m_armAngle.getEncoder();

      e_armExtend.setPosition(0.0);
      e_armAngle.setPosition(0.0);

      ce_initPos = e_armExtend.getPosition();
      ca_initPos = e_armAngle.getPosition();

      ca_integral  = 0.0;
      ca_errorPrev = 0.0;
   }

   public void setArmAngle(int position) {
      double ca_cmd;
      double ca_error;
      double angleTgt = 0;
       // 0 - initial retracted position
      // 1 - lowest coral placement
      // 2 - mid coral placement
      // 3 - highest position
      
      if (position==ArmConstants.TGT_INIT) angleTgt = ca_initPos;
      if (position==ArmConstants.TGT_LOW)  angleTgt = ca_initPos + ArmConstants.lowTgtAngle;
      if (position==ArmConstants.TGT_MID)  angleTgt = ca_initPos + ArmConstants.midTgtAngle;
      if (position==ArmConstants.TGT_HIGH) angleTgt = ca_initPos + ArmConstants.highTgtAngle;

      ////// CORAL ROTATION ANGLE LOGIC using PI control - goal is to drive error to zero      
      ca_error    = angleTgt - e_armAngle.getPosition();  //calculate the error between desired and current
      ca_integral = ca_integral + ca_errorPrev;           // this is the error added up over time
      ca_cmd      = ca_error*ArmConstants.CA_PGain + ca_integral*ArmConstants.CA_IGain; //both P and I terms combined
    
      // This logic just saturates the command making sure it doesn't get too strong
      if (ca_cmd >  ArmConstants.CA_MAX) {ca_cmd =  ArmConstants.CA_MAX;}
      if (ca_cmd < -ArmConstants.CA_MAX) {ca_cmd = -ArmConstants.CA_MAX;}
  
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
      
      if (position==ArmConstants.TGT_INIT) extendTgt = ce_initPos;
      if (position==ArmConstants.TGT_LOW)  extendTgt = ce_initPos + ArmConstants.lowTgtHeight;
      if (position==ArmConstants.TGT_MID)  extendTgt = ce_initPos + ArmConstants.midTgtHeight;
      if (position==ArmConstants.TGT_HIGH) extendTgt = ce_initPos + ArmConstants.highTgtHeight;
      
       ////// CORAL EXTENSION LENGTH uses only P control - goal is to drive error to zero  
      ce_error = extendTgt - e_armExtend.getPosition();  //calculate the error between desired and current
      ce_cmd = ArmConstants.CE_PGain * ce_error;         //error times P gain gives command
      
      // This logic just saturates the command making sure it doesn't get too strong
      if (ce_cmd >  ArmConstants.CE_MAX) {ce_cmd =  ArmConstants.CE_MAX;}
      if (ce_cmd < -ArmConstants.CE_MAX) {ce_cmd = -ArmConstants.CE_MAX;}

      m_armExtend.set(ce_cmd);
   }

   public void stopArm() {
      m_armExtend.stopMotor();
      m_armAngle.stopMotor();
   }

   @Override
   public void periodic() {
      // This method will be called once per scheduler run
      // You can use it for telemetry, diagnostics, etc.
   }
}
