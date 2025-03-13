// File: ResetFieldOrientationCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LedSubsystem;

public class ResetFieldOrientationCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final LedSubsystem ledSubsystem;
  private final double timeoutSeconds = 15.0;
  private double startTime;

  public ResetFieldOrientationCommand(DriveSubsystem driveSubsystem,
                                        VisionSubsystem visionSubsystem,
                                        LedSubsystem ledSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.ledSubsystem = ledSubsystem;
    addRequirements(driveSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {
    // Record the start time (in seconds)
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    // Check if a valid tag is detected (assume valid if detected tag ID > 0)
    if (visionSubsystem.getDetectedTagIDFromNT() > 0) {
      // Reset the field orientation using the current heading.
      driveSubsystem.resetFieldOrientation();
      // Set LEDs to solid alliance color.
      ledSubsystem.setLEDMode(LedSubsystem.LEDMode.SOLID_GREEN);
    } else {
      // No valid tag detected: flash green.
      ledSubsystem.setLEDMode(LedSubsystem.LEDMode.FLASH_GREEN);
    }
  }

  @Override
  public boolean isFinished() {
    // End if a valid tag is detected or if 15 seconds have passed.
    return (visionSubsystem.getDetectedTagIDFromNT() > 0) ||
           (Timer.getFPGATimestamp() - startTime >= timeoutSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    // Ensure LEDs are set to solid once the command ends.
    ledSubsystem.setLEDMode(LedSubsystem.LEDMode.SOLID_GREEN);
  }
}
