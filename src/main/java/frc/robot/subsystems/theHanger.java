// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class theHanger extends SubsystemBase {
  /** Creates a new theHanger. */
  
  private SparkMax harpoon = new SparkMax(ArmConstants.HarpoonCanId,MotorType.kBrushless);
  RelativeEncoder harpoonEncoder= harpoon.getEncoder();
  boolean end;
  boolean check=true;
  
  // Init
  public theHanger() {
    end=false;
    if (check){
    harpoonEncoder.setPosition(0);//
    check=!check;
    }
  }

  public boolean setArmPos() {
  if ((10>=harpoonEncoder.getPosition())&&(harpoonEncoder.getPosition()>= -334)){
      if (harpoonEncoder.getPosition()<=-335){
        harpoon.set(0);
        end=true;
      }else {
        harpoon.set(-1);
      }
} else if((-335>=harpoonEncoder.getPosition())&&(harpoonEncoder.getPosition()>= -359)) {
      if (harpoonEncoder.getPosition()<=-360){
        harpoon.set(0);
        end=true;
      }else {
        harpoon.set(-1);
      }
    } else if (harpoonEncoder.getPosition()<=-360) {
      if (harpoonEncoder.getPosition()>=0){
          harpoon.set(0);
          end=true;
      }else {
          harpoon.set(1);
      }
        }
          return end;
          }


          public void manualHaroonUp (boolean speed)
          {
            if (speed) {
              harpoon.set(-.6);
            } else {
              harpoon.set(-1);
            };
          }

          public void manualHaroonDown (boolean speed){
          if (speed) {
            harpoon.set(.6);
          } else {
            harpoon.set(1);
          };

          }
          public void harpoonStoop(){
            harpoon.set(.0);
          };

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
