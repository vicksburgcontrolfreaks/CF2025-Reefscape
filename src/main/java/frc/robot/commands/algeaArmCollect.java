// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algeaArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class algeaArmCollect extends Command {
  /** Creates a new greenArmRun. */
  private algeaArm springArm; 
  public algeaArmCollect() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(springArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    springArm.algeaArmCollect();
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //ends the command when the end variable if greenArm=true
    return springArm.algeaArmCollect();
  }
   // Called once the command ends or is interrupted.
   @Override
   public void end (boolean interrupted) {
   }
}