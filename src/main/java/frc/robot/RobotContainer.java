//  Control Freaks 2025

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.OscillateDistanceCommand;
import frc.robot.commands.TrajectoryAutoCommand;
import frc.robot.commands.TrajectoryToTagCommand;
import frc.robot.commands.raiseCoralArm;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlgaeCollectorSubsystem;
import frc.robot.subsystems.AlgaeExtenderSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotContainer {
   // The robot's subsystems
   private final DriveSubsystem m_robotDrive = new DriveSubsystem();
   private final AlgaeCollectorSubsystem m_algaeCollector = new AlgaeCollectorSubsystem();
   private final AlgaeExtenderSubsystem m_algaeExtender = new AlgaeExtenderSubsystem();
   private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
   private final LocalizationSubsystem m_localizationSubsystem = new LocalizationSubsystem(m_robotDrive);
   private final CoralArmSubsystem m_coralArmSubsystem = new CoralArmSubsystem();

   // The driver's controller (for driving)
   private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
   // The mechanism controller (for robot mechanisms)
   private final XboxController m_mechanismController = new XboxController(OIConstants.kMechanismControllerPort);

   // Create a chooser for autonomous routines.
   public final SendableChooser<Command> autoChooser = new SendableChooser<>();

   public RobotContainer() {
      //Determine alliance assignment
      DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
      if (alliance == DriverStation.Alliance.Blue) {
          // Set up autonomous command for blue alliance.
      } else if (alliance == DriverStation.Alliance.Red) {
          // Set up autonomous command for red alliance.
      } else {
          // This branch will likely never be reached since only Red and Blue exist.
      }

      // Configure button bindings
      configureButtonBindings();
      // Publish the VisionSubsystem to SmartDashboard for monitoring.
      SmartDashboard.putData("Vision Subsystem", m_visionSubsystem);

      // Configure default teleop command: drive the robot.
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
      autoChooser.setDefaultOption("Competition Ready Auton", new OscillateDistanceCommand(m_robotDrive));
      autoChooser.addOption("Trajectory Auto", new TrajectoryAutoCommand(m_robotDrive));
      autoChooser.addOption("Forward Tune", new OscillateDistanceCommand(m_robotDrive));
      // (Optional) Add a "Do Nothing" option.
      autoChooser.addOption("No Auto",
         new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive));

      // Publish the autonomous chooser to SmartDashboard.
      SmartDashboard.putData("Auto Tuning Mode", autoChooser);
   }

   private void configureButtonBindings() {
      // ******************************** Driver controller ***********************************************
      new JoystickButton(m_driverController, Button.kR1.value)
         .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

         POVButton dpadLeftButton = new POVButton(m_driverController, 270);
         POVButton dpadUpButton = new POVButton(m_driverController, 0);
         POVButton dpadRightButton = new POVButton(m_driverController, 90);
         POVButton dpadDownButton = new POVButton(m_driverController, 180);

         // Auto-align to April tag with driver controller.
         dpadLeftButton.onTrue(new TrajectoryToTagCommand(m_robotDrive, m_visionSubsystem, true));
         dpadRightButton.onTrue(new TrajectoryToTagCommand(m_robotDrive, m_visionSubsystem, false));
         dpadDownButton.onTrue(new InstantCommand(m_robotDrive::toggleFieldOriented, m_robotDrive));

         // ********************************* Mech controller ************************************************
         new JoystickButton(m_mechanismController, XboxController.Button.kA.value)
            .whileTrue(new RunCommand(() -> {
               m_algaeCollector.moveArm(0.5);}, m_algaeCollector))  // Start the collector
               .whileFalse(new InstantCommand(() -> m_algaeCollector.stopArm(), m_algaeCollector)); // Stop when released

         new JoystickButton(m_mechanismController, XboxController.Button.kX.value)
            .whileTrue(new RunCommand(() -> {
               m_algaeExtender.moveArm(m_algaeExtender.getInitPos()+10);}, m_algaeExtender)); 

         new JoystickButton(m_mechanismController, XboxController.Button.kY.value)
            .whileTrue(new RunCommand(() -> {
               m_algaeExtender.moveArm(m_algaeExtender.getInitPos());}, m_algaeExtender)); 

         POVButton mech_padLeftButton = new POVButton(m_mechanismController, 270);
         POVButton mech_dpadUpButton = new POVButton(m_mechanismController, 0);
         POVButton mech_dpadRightButton = new POVButton(m_mechanismController, 90);
         POVButton mech_dpadDownButton = new POVButton(m_mechanismController, 180);

         // Coral Arm
         // Bind D-pad up to raise the arm. Positive speed to raise.
         mech_dpadUpButton.whileTrue(new raiseCoralArm(m_coralArmSubsystem, 0.05));
         // Bind D-pad down to lower the arm. Negative speed to lower.
         mech_dpadDownButton.whileTrue(new raiseCoralArm(m_coralArmSubsystem, -0.05));
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
