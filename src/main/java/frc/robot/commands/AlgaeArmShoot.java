// 2025 Control Freaks

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;

public class AlgaeArmShoot extends Command {
  /** Creates a new algeaArmIn. */
  private AlgaeArmSubsystem springArm; 
  public AlgaeArmShoot() {
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
    springArm.algaeArmShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public boolean isFinished() {
    return springArm.algaeArmShoot();
  }

  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
 
}