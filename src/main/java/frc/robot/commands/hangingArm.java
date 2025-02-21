// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.theHanger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class hangingArm extends Command {
  /** Creates a new hangingArm. */
   private theHanger harpoonArm;
  public hangingArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(harpoonArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  harpoonArm.setArmPos();
  }

  // Called once the command ends or is interrupted.
  @Override
  public boolean isFinished() {
   return harpoonArm.setArmPos();
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
 
}
