// HomeCoralArmCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.NewCoralArmSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HomeCoralArmCommand extends Command {
    private final NewCoralArmSubsystem m_armSubsystem;
   
    private static final double TOL_INCREMENT = 0.002;
    private static final double ANG_TOL_INC   = 0.0002;
    private static final double POSITION_TOLERANCE = 0.2; // acceptable error
    private static final double ANG_POS_TOL = 0.02;
    private double tolerance_offset;
    private double ang_tol_off;


    /**
     * Constructs a command to retract the coral arm to its home position.
     *
     * @param armSubsystem The coral arm subsystem.
     */
    public HomeCoralArmCommand(NewCoralArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        // Optionally reset integrators here.
        tolerance_offset = 0;
        ang_tol_off = 0.0;
    }

    @Override
    public void execute() {
        // Command the arm to move to home (0 position).
  

        m_armSubsystem.moveArm(ArmConstants.TGT_INIT);



        // make sure the extension is back before angling into home
        // to avoid contact with intake
        if (m_armSubsystem.getCurrentExtension() > -2.0) {
            m_armSubsystem.setArmAngle(0.0); //Home is zero
            tolerance_offset = tolerance_offset + TOL_INCREMENT;
            ang_tol_off = ang_tol_off + ANG_TOL_INC;
        } else {
            tolerance_offset = 0;
            ang_tol_off = 0.0;
        }

        // Publish current measurements for debugging.
        double angleCurrent = m_armSubsystem.getAngleCurrent();
        double extendCurrent = m_armSubsystem.getExtendCurrent();
        // SmartDashboard.putNumber("Arm Angle Current", angleCurrent);
        // SmartDashboard.putNumber("Arm Extend Current", extendCurrent);
    }

    @Override
    public boolean isFinished() {
        //boolean hardStop = (m_armSubsystem.getAngleCurrent() > CURRENT_THRESHOLD ||
        //                      m_armSubsystem.getExtendCurrent() > CURRENT_THRESHOLD);
        double currentExtension = m_armSubsystem.getCurrentExtension();
        double currentAngle = m_armSubsystem.getArmAngle();
        boolean nearHome = (Math.abs(currentExtension - m_armSubsystem.getInitArmExtend()) < (POSITION_TOLERANCE+tolerance_offset));
        boolean angleNearHome = (Math.abs(currentAngle - m_armSubsystem.getInitArmAngle()) < (ANG_POS_TOL+ang_tol_off));
        return angleNearHome && nearHome;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopArm();
    }
}
