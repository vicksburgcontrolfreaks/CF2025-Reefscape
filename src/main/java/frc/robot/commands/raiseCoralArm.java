
package frc.robot.commands;

//import com.pathplanner.lib.PathPlanner;
import frc.robot.subsystems.CoralArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class raiseCoralArm extends Command {
    private final CoralArmSubsystem armSubsystem;

    private final int position;

    public raiseCoralArm(CoralArmSubsystem armSubsystem, int position) {
        this.armSubsystem = armSubsystem;
        this.position = position;
        addRequirements(armSubsystem); // Ensures that the arm subsystem is required for this command
    }

    @Override
    public void execute() {
        armSubsystem.moveArm(position);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopArm();
    }

    @Override
    public boolean isFinished() {
        return false; // If you want it to be continuous, return false
    }
}
