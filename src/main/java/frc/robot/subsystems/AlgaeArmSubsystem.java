// Control Freaks 2025

package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArmSubsystem extends SubsystemBase {

   private SparkMax greenSpring  = new SparkMax(ArmConstants.AlgaeExtenderCanId,MotorType.kBrushless);
   private SparkMax greenWheel  = new SparkMax(ArmConstants.AlgaeCollectorCanId, MotorType.kBrushless);
   double springStopEndPos;
   final static double wheelStopSoftCap = 5.0;
   double greenWheelCounter;
   boolean springStartup;
   RelativeEncoder springEncoder = greenSpring.getEncoder(); 
   boolean end;
   boolean check=true;
   
   //Init
   public AlgaeArmSubsystem() {
      end=false;
      if (check){
         greenWheelCounter=0; 
         springEncoder.setPosition(0);
         springStartup=false;
         check=!check;
      }
   }

   public boolean algaeArmCollect () {
      //checks if the timer for spring arm is done if not adds 1 to the counter
      if (springEncoder.getPosition()>=springStopEndPos){
         greenSpring.set(0);
      } else {
         greenSpring.set(-1);
      }

    //checks for the greenwheeloutputcurrent if it sees a algea is collected starts a chain reacion to end the subsystem and the command
    
      if (greenWheel.getOutputCurrent()>= wheelStopSoftCap){
         end=true;
      }
      // puts the wheel speed to zero 
      if (end){
         greenWheel.set(0);
      } else {
         greenWheel.set(-1);
      } 
      // returns true or false based on if the last thing that happens in the function the wheel stop is true

      return end;
   }
   public boolean algaeArmShoot () {
      //checks if the timer for spring arm is done if not adds 1 to the counter
      if (greenWheelCounter>=2000){
         greenWheel.set(0);
         springStartup=true;
      } else {
         greenWheel.set(1);
         greenWheelCounter++;
      }
      if (springStartup){
         greenWheel.getLastError();
         if (springEncoder.getPosition()<=0){
            greenSpring.set(0);
            greenWheelCounter=0; 
            springStartup=false;
            end=true;
         } else {
            greenSpring.set(1);
         }
      }
      return end;
   }

   @Override
   public void periodic() {
   }
}