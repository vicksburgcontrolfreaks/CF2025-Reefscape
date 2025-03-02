package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewCoralArmSubsystem;

public class SetArmPositionCommand extends Command {
    private final NewCoralArmSubsystem armSubsystem;
    private final double targetAngle;
    private final double targetExtension;
    private static final double BIG_TOLERANCE = 4.0;
    private static final double TOL_INCREMENT = 0.002;
    private static final double TOLERANCE = 1.0; // Tune as needed
    private double currentAngle;
    private double tolerance_offset;
    
    public SetArmPositionCommand(NewCoralArmSubsystem armSubsystem, double targetAngle, double targetExtension) {
        this.armSubsystem = armSubsystem;
        this.targetAngle = targetAngle;
        this.targetExtension = targetExtension;
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
        // Optionally reset control integrals here if needed.
        tolerance_offset = 0.0;
    }
    
    @Override
    public void execute() {

        currentAngle = armSubsystem.getArmAngle();
        // if it is close to target, start increasing the small tolerance
        if (Math.abs(currentAngle - targetAngle) < BIG_TOLERANCE) {
            tolerance_offset = tolerance_offset + TOL_INCREMENT;
            if (tolerance_offset > 2.0) tolerance_offset = 2.0;
        } else {
            tolerance_offset = 0;
        }

        armSubsystem.setArmAngle(targetAngle);
        if (armSubsystem.getArmAngle() > armSubsystem.getInitArmAngle() + 8) {
           armSubsystem.moveArm(targetExtension);
        }
    }
    
    @Override
    public boolean isFinished() {
        double currentAngle = armSubsystem.getArmAngle();
        double currentExtension = armSubsystem.getCurrentExtension();
        return Math.abs(currentAngle - targetAngle) < (TOLERANCE+tolerance_offset) &&
               Math.abs(currentExtension - targetExtension) < (TOLERANCE+tolerance_offset);
    }
    
    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopArm();
    }
}
