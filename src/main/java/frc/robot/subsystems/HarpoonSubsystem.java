// Control Freaks 2025
// Harpoon subsystem controls climbing device

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.RelativeEncoder;

public class HarpoonSubsystem extends SubsystemBase {
   private RelativeEncoder e_harpoon;
   private final SparkMax m_harpoon;
   private double initPos;
   //private double cmd;
   //private double PGain = 0.05;
   //private double currPos;

   public HarpoonSubsystem() {
      m_harpoon = new SparkMax(ArmConstants.HarpoonCanId, MotorType.kBrushless);
      e_harpoon = m_harpoon.getEncoder();
      e_harpoon.setPosition(0.0);
      initPos = e_harpoon.getPosition();
      SmartDashboard.putNumber("H Init",initPos);
   }

   public double getInitPos() {
      return(initPos);
   }

   public void moveArm(double targetPos) {
      //currPos = e_algaeExtender.getPosition();
      //cmd = PGain * (targetPos - currPos);
      //SmartDashboard.putNumber("AE Pos",currPos); 
      //SmartDashboard.putNumber("AE Cmd",cmd);
      //SmartDashboard.putNumber("AE Tgt",targetPos);
      //if (cmd > 0.3) cmd = 0.25;
      //if (cmd < -0.3) cmd = -0.19;;
      //m_algaeExtender.set(cmd);
   }

   public void stopArm() {
      //m_algaeExtender.stopMotor();
   }

   @Override
   public void periodic() {
      // This method will be called once per scheduler run
      // You can use it for telemetry, diagnostics, etc.
   }
}
