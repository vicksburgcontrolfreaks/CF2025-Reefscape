// File: AutoTuneLateralCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoTuneLateralCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    public AutoTuneLateralCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        // Publish default tuning values for lateral control.
        SmartDashboard.putNumber("Lateral kP", AutoConstants.kPLateral);
        // Use one of your calibrated setpoints; for instance, for left alignment:
        SmartDashboard.putNumber("Desired TX", AutoConstants.kLeftAlignTX);
        SmartDashboard.putNumber("Lateral Tolerance", AutoConstants.kLateralTolerance);
    }

    @Override
    public void execute() {
        // Retrieve PID constant and desired tx from SmartDashboard.
        double kP = SmartDashboard.getNumber("Lateral kP", AutoConstants.kPLateral);
        double desiredTX = SmartDashboard.getNumber("Desired TX", AutoConstants.kLeftAlignTX);
        double tolerance = SmartDashboard.getNumber("Lateral Tolerance", AutoConstants.kLateralTolerance);

        double currentTX = visionSubsystem.getTx();
        double error = desiredTX - currentTX;
        double output = kP * error;

        // Drive laterally (assuming positive output moves the robot sideways).
        driveSubsystem.drive(0, output, 0, true);

        // Publish diagnostic data.
        SmartDashboard.putNumber("Lateral Error", error);
        SmartDashboard.putNumber("Lateral Output", output);
    }

    @Override
    public boolean isFinished() {
        double tolerance = SmartDashboard.getNumber("Lateral Tolerance", AutoConstants.kLateralTolerance);
        double error = Math.abs(SmartDashboard.getNumber("Desired TX", AutoConstants.kLeftAlignTX) - visionSubsystem.getTx());
        return error < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
    }
}
