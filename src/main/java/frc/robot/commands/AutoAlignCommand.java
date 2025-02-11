package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;

public class AutoAlignCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final boolean alignLeft;
    
    // PID controllers for forward and lateral control
    private final PIDController forwardPID;
    private final PIDController lateralPID;
    
    public AutoAlignCommand(DriveSubsystem drive, VisionSubsystem vision, boolean alignLeft) {
        this.driveSubsystem = drive;
        this.visionSubsystem = vision;
        this.alignLeft = alignLeft;
        
        forwardPID = new PIDController(AutoConstants.kPForward, 0, 0);
        lateralPID = new PIDController(AutoConstants.kPLateral, 0, 0);
        
        addRequirements(driveSubsystem, visionSubsystem);
    }
    
    @Override
    public void initialize() {
        forwardPID.reset();
        lateralPID.reset();
    }
    
    @Override
    public void execute() {
        // Get current vision data.
        double tx = visionSubsystem.getTx();
        double ta = visionSubsystem.getTa();
        
        // Choose desired tx offset based on which alignment mode is selected.
        double desiredTx = alignLeft ? AutoConstants.kLeftAlignTX : AutoConstants.kRightAlignTX;
        
        // Calculate errors.
        // For lateral alignment, error is the difference between the desired tx offset and the measured tx.
        double lateralError = desiredTx - tx;
        // For forward alignment, error is the difference between the desired target area and the measured area.
        double forwardError = AutoConstants.kTargetArea - ta;
        
        // Compute the PID outputs.
        double forwardOutput = forwardPID.calculate(ta, AutoConstants.kTargetArea);
        double lateralOutput = lateralPID.calculate(tx, desiredTx);
        
        // Drive the robot.
        // Here, we assume that the drive subsystem's drive() method takes:
        // forward speed (x), lateral speed (y), rotational speed, and a fieldRelative boolean.
        driveSubsystem.drive(forwardOutput, lateralOutput, 0, true);
    }
    
    @Override
    public boolean isFinished() {
        double tx = visionSubsystem.getTx();
        double ta = visionSubsystem.getTa();
        
        double lateralError = Math.abs((alignLeft ? AutoConstants.kLeftAlignTX : AutoConstants.kRightAlignTX) - tx);
        double forwardError = Math.abs(AutoConstants.kTargetArea - ta);
        
        return lateralError < AutoConstants.kLateralTolerance && forwardError < AutoConstants.kForwardTolerance;
    }
    
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
    }
}
