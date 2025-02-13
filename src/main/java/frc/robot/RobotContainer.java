// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.OscillateDistanceCommand;
import frc.robot.commands.TrajectoryAutoCommand;
import frc.robot.commands.TrajectoryToTagCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
        private final LocalizationSubsystem m_localizationSubsystem = new LocalizationSubsystem(m_robotDrive);

        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

        // Create a chooser for autonomous routines.
        public final SendableChooser<Command> autoChooser = new SendableChooser<>();

        public RobotContainer() {
                // Configure button bindings
                configureButtonBindings();

                // Publish the VisionSubsystem to SmartDashboard for monitoring.
                SmartDashboard.putData("Vision Subsystem", m_visionSubsystem);

                // Configure default teleop command: drive the robot
                m_robotDrive.setDefaultCommand(
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getLeftY() * 0.5,
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getLeftX() * 0.5,
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRightX() * 0.5,
                                                                                OIConstants.kDriveDeadband),
                                                                true),
                                                m_robotDrive));

                // Set up autonomous tuning options.
                autoChooser.setDefaultOption("Compeition Ready Auton", new OscillateDistanceCommand(m_robotDrive));
                autoChooser.addOption("Trajectory Auto", new TrajectoryAutoCommand(m_robotDrive));
                autoChooser.addOption("Forward Tune", new OscillateDistanceCommand(m_robotDrive));
                // (Optional) Add a "Do Nothing" option.
                autoChooser.addOption("No Auto",
                                new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive));

                // for choosing autonomous mode
                SmartDashboard.putData("Auto Tuning Mode", autoChooser);
        }

        private void configureButtonBindings() {
                new JoystickButton(m_driverController, Button.kR1.value)
                                .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

                POVButton dpadLeftButton = new POVButton(m_driverController, 270);
                POVButton dpadUpButton = new POVButton(m_driverController, 0);
                POVButton dpadRightButton = new POVButton(m_driverController, 90);
                POVButton dpadDownButton = new POVButton(m_driverController, 180);

                // Autoalign to April tag
                dpadLeftButton.onTrue(new TrajectoryToTagCommand(m_robotDrive, m_visionSubsystem, true));
                dpadRightButton.onTrue(new TrajectoryToTagCommand(m_robotDrive, m_visionSubsystem, false));
                dpadDownButton.onTrue(new InstantCommand(m_robotDrive::toggleFieldOriented, m_robotDrive));

        }

        /**
         * This method is called by the main Robot class to get the command to run in
         * autonomous.
         */
        public Command getAutonomousCommand() {
                // Return the autonomous command selected from the chooser.
                return autoChooser.getSelected();
        }
}