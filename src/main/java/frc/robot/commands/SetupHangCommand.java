package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.HarpoonSubsystem;
import frc.robot.subsystems.NewCoralArmSubsystem;

public class SetupHangCommand extends SequentialCommandGroup {

    private double WaitTime = 0.1;

    public SetupHangCommand(
        HarpoonSubsystem harpoon,
        NewCoralArmSubsystem coralArm,
        AlgaeArmSubsystem algaeArm) {
        
        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> coralArm.setArmAngle(0.10), coralArm),
                new WaitCommand(WaitTime),
                new InstantCommand(() -> coralArm.moveArm(-20), coralArm),
                new WaitCommand(WaitTime),
                new InstantCommand(() -> coralArm.setArmAngle(0), coralArm),
                
                new WaitCommand(WaitTime),

                new InitAlgaeCollectorPositionCommand(algaeArm, 20),

                new WaitCommand(WaitTime),

                new InstantCommand(() -> harpoon.setMotor(0.5), harpoon),
                new WaitCommand(2.0),
                new InstantCommand(() -> harpoon.stop(), harpoon)
            )
        );
    }
}
