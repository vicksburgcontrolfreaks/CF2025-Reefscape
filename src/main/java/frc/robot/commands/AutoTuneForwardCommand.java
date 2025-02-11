// File: AutoTuneForwardCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoTuneForwardCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    public AutoTuneForwardCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        // Publish default tuning values so you can adjust them on SmartDashboard.
        SmartDashboard.putNumber("Forward kP", AutoConstants.kPForward);
        SmartDashboard.putNumber("Target Area", AutoConstants.kTargetArea);
        SmartDashboard.putNumber("Forward Tolerance", AutoConstants.kForwardTolerance);
    }

    @Override
    public void execute() {
        // Retrieve PID constant and desired target area from SmartDashboard.
        double kP = SmartDashboard.getNumber("Forward kP", AutoConstants.kPForward);
        double targetArea = SmartDashboard.getNumber("Target Area", AutoConstants.kTargetArea);
        double tolerance = SmartDashboard.getNumber("Forward Tolerance", AutoConstants.kForwardTolerance);

        double currentArea = visionSubsystem.getTa();
        double error = targetArea - currentArea;
        double output = kP * error;

        // Drive forward (assuming positive output drives forward).
        driveSubsystem.drive(output, 0, 0, true);

        // Publish diagnostic data.
        SmartDashboard.putNumber("Forward Error", error);
        SmartDashboard.putNumber("Forward Output", output);
    }

    @Override
    public boolean isFinished() {
        double tolerance = SmartDashboard.getNumber("Forward Tolerance", AutoConstants.kForwardTolerance);
        double error = Math.abs(SmartDashboard.getNumber("Target Area", AutoConstants.kTargetArea) - visionSubsystem.getTa());
        return error < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
    }
}
