package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewCoralArmSubsystem;

public class SetArmPositionCommand extends Command {
    private final NewCoralArmSubsystem armSubsystem;
    private final double targetAngle;
    private final double targetExtension;
    private static final double TOLERANCE = 1.0; // Tune as needed
    
    public SetArmPositionCommand(NewCoralArmSubsystem armSubsystem, double targetAngle, double targetExtension) {
        this.armSubsystem = armSubsystem;
        this.targetAngle = targetAngle;
        this.targetExtension = targetExtension;
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
        // Optionally reset control integrals here if needed.
    }
    
    @Override
    public void execute() {
        armSubsystem.setArmAngle(targetAngle);
        if (armSubsystem.getArmAngle() > armSubsystem.getInitArmAngle() + 8) {
           armSubsystem.moveArm(targetExtension);
        }
    }
    
    @Override
    public boolean isFinished() {
        double currentAngle = armSubsystem.getArmAngle();
        double currentExtension = armSubsystem.getCurrentExtension();
        return Math.abs(currentAngle - targetAngle) < TOLERANCE &&
               Math.abs(currentExtension - targetExtension) < TOLERANCE;
    }
    
    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopArm();
    }
}
