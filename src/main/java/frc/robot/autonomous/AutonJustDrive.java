package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutonJustDrive extends SequentialCommandGroup {
    public AutonJustDrive(DriveSubsystem driveSubsystem) {
        addCommands(
            // Reset odometry to ensure accurate movement
            new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0))), driveSubsystem),
            new WaitCommand(0.5),
            // Drive forward 1 meter
            new InstantCommand(() -> driveSubsystem.drive(-0.5, 0.0, 0.0, true), driveSubsystem),
            new WaitCommand(0.5), // Adjust timing if necessary
            new InstantCommand(() -> driveSubsystem.drive(0.0, 0.0, 0.0, true), driveSubsystem)
        );
    }
}