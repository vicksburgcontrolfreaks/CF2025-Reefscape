// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoTuneForwardCommand;
import frc.robot.commands.AutoTuneLateralCommand;
import frc.robot.commands.TrajectoryAutoCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.TelemetrySubsystem;

public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    private final LocalizationSubsystem m_localizationSubsystem = new LocalizationSubsystem(m_robotDrive, m_visionSubsystem);
    
    // Create a chooser for autonomous routines.
    public final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // PUT ALL SMART DASHBOARD STUFF IN THIS TO KEEP STUFF ORGANIZED
    // NOTE: edit the arguments to pass stuff through to the subsystem DO NOT MAKE IT PUBLIC MAINTAIN MEMORY INTEGRITY
    private final TelemetrySubsystem m_telemetrySubsystem = new TelemetrySubsystem(m_visionSubsystem, autoChooser);

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);


    public RobotContainer() {
        // Configure button bindings
        configureButtonBindings();

        // Configure default teleop command: drive the robot
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY() * 0.5,
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX() * 0.5,
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX() * 0.5,
                                        OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));

        // Set up autonomous tuning options.
        autoChooser.setDefaultOption("Forward Tuning", new AutoTuneForwardCommand(m_robotDrive, m_visionSubsystem));
        autoChooser.addOption("Lateral Tuning", new AutoTuneLateralCommand(m_robotDrive, m_visionSubsystem));

        // Add your new trajectory-following autonomous command.
        autoChooser.addOption("Trajectory Auto", new TrajectoryAutoCommand(m_robotDrive));

        // (Optional) Add a "Do Nothing" option.
        autoChooser.addOption("No Auto", new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive));

        // PUT ALL SMART DASHBOARD STUFF IN THIS TO KEEP STUFF ORGANIZED
        m_telemetrySubsystem.ConfigureDashboard();
    }

    private void configureButtonBindings() {
        new JoystickButton(m_driverController, Button.kR1.value)
                .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

        // Auto-align left when button 1 (A button) is held.
        new JoystickButton(m_driverController, 1) // Replace 1 with the appropriate button value.
                .whileTrue(new AutoAlignCommand(m_robotDrive, m_visionSubsystem, true));

        // Auto-align right when button 2 (B button) is held.
        new JoystickButton(m_driverController, 2) // Replace 2 with the appropriate button value.
                .whileTrue(new AutoAlignCommand(m_robotDrive, m_visionSubsystem, false));
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