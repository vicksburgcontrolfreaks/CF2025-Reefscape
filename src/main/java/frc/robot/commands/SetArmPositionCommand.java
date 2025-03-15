package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewCoralArmSubsystem;

public class SetArmPositionCommand extends Command {
    private final NewCoralArmSubsystem armSubsystem;
    private final double targetAngle;
    private final double targetExtension;
    private static final double BIG_ANG_TOL = 0.04;

    private static final double EXT_TOL_INCREMENT = 0.002;
    private static final double ANG_TOL_INCREMENT = 0.0002;

    private static final double ANG_TOL = 0.01;
    private static final double EXT_TOL = 1.0;
    
    private double targetAngleModified;
    private double currentAngle;
    private double currentExtension;
    private double ang_tol_off;
    private double ext_tol_off;
    private double ang_error;
    private double ext_error;
    
    public SetArmPositionCommand(NewCoralArmSubsystem armSubsystem, double targetAngle, double targetExtension) {
        this.armSubsystem = armSubsystem;
        this.targetAngle = targetAngle;
        this.targetExtension = targetExtension;
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
        // Optionally reset control integrals here if needed.
        //tolerance_offset = 0.0;
        ang_tol_off = 0.0;
        ext_tol_off = 0.0;
        ang_error = 0.0;
        ext_error = 0.0;
    }
    
    @Override
    public void execute() {

        currentAngle = armSubsystem.getArmAngle();
        ang_error = (targetAngle - currentAngle);

        currentExtension = armSubsystem.getCurrentExtension();
        ext_error = (targetExtension - currentExtension);

        // if it is close to target, start increasing the small tolerance
        if (ang_error < BIG_ANG_TOL) {
            ang_tol_off =  ang_tol_off + ANG_TOL_INCREMENT;
            ext_tol_off = ext_tol_off + EXT_TOL_INCREMENT;
            if (ang_tol_off > 0.05) ang_tol_off = 0.05;
            if (ext_tol_off > 2.0) ext_tol_off = 2.0;
        } else {
            ang_tol_off = 0.0;
            ext_tol_off = 0.0;
        }

        if (targetAngle > 0.3) {
            targetAngleModified = 0.3;
            if (Math.abs(armSubsystem.getCurrentExtension()) > 30) {
                targetAngleModified = targetAngle;      
            }
        } else {
            targetAngleModified = targetAngle;
        }
        armSubsystem.setArmAngle(targetAngleModified);
        // Don't move extension out until past intake
        if (armSubsystem.getArmAngle() > armSubsystem.getInitArmAngle() + 0.08) {
           armSubsystem.moveArm(targetExtension);
        }

        // SmartDashboard.putNumber("ANG_tgt", targetAngle); 
        // SmartDashboard.putNumber("ANG_cur", currentAngle);   //targetAngle
        // SmartDashboard.putNumber("ANG_TOL", ANG_TOL + ang_tol_off);
        //SmartDashboard.putNumber("EXT_TOL", EXT_TOL + ext_tol_off);
        // SmartDashboard.putNumber("ang_err", ang_error);
        //SmartDashboard.putNumber("ext_err", ext_error);
    }
    
    @Override
    public boolean isFinished() {
        return ((Math.abs(ang_error) < (ANG_TOL+ang_tol_off)) &&
                (Math.abs(ext_error) < (EXT_TOL+ext_tol_off)));
    }
    
    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopArm();
    }
}
