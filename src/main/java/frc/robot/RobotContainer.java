//  Control Freaks 2025

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.OscillateDistanceCommand;
import frc.robot.autonomous.TrajectoryAutoCommand;
import frc.robot.commands.DriveToTagCommand;
import frc.robot.commands.HomeCoralArmCommand;
import frc.robot.commands.ManualCoralArmAdjustCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.subsystems.NewCoralArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlgaeCollectorSubsystem;
import frc.robot.subsystems.AlgaeExtenderSubsystem;
import frc.robot.subsystems.HarpoonSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotContainer {
   // Determine alliance assignment
   DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

   // The robot's subsystems
   private final DriveSubsystem m_robotDrive = new DriveSubsystem();
   private final AlgaeCollectorSubsystem m_algaeCollector = new AlgaeCollectorSubsystem();
   private final AlgaeExtenderSubsystem m_algaeExtender = new AlgaeExtenderSubsystem();
   private final NewCoralArmSubsystem m_coralArmSubsystem = new NewCoralArmSubsystem();
   private final HarpoonSubsystem m_harpoon = new HarpoonSubsystem();
   private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
   private final LocalizationSubsystem m_localizationSubsystem = new LocalizationSubsystem(m_robotDrive);

   // Use the enum for arm positions.
   public enum ArmPosition {
      INIT, LOW, MID, HIGH;
   }

   private ArmPosition currentArmPosition = ArmPosition.INIT;
   private ArmPosition targetArmPosition = ArmPosition.INIT;

   // Variable to track scoring side (true = left, false = right); default left.
   private boolean m_scoringSideLeft = true;

   // The driver's controller (for driving)
   private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
   POVButton dpadUpButton = new POVButton(m_driverController, 0);
   POVButton dpadRightButton = new POVButton(m_driverController, 90);
   POVButton dpadDownButton = new POVButton(m_driverController, 180);
   POVButton dpadLeftButton = new POVButton(m_driverController, 270);

   // The mechanism controller (for robot mechanisms)
   private final XboxController m_mechanismController = new XboxController(OIConstants.kMechanismControllerPort);
   POVButton mech_dpadUpButton = new POVButton(m_mechanismController, 0);
   POVButton mech_dpadRightButton = new POVButton(m_mechanismController, 90);
   POVButton mech_dpadDownButton = new POVButton(m_mechanismController, 180);
   POVButton mech_dpadLeftButton = new POVButton(m_mechanismController, 270);

   // Create a chooser for autonomous routines.
   public final SendableChooser<Command> autoChooser = new SendableChooser<>();
   // Field to store the currently scheduled arm command.
   private Command m_currentArmCommand = null;

   public RobotContainer() {
      //
      SmartDashboard.putData("Field", m_localizationSubsystem.getField());

      // Auton Settings
      if (alliance == DriverStation.Alliance.Blue) {
         // Set up autonomous command for blue alliance.
      } else if (alliance == DriverStation.Alliance.Red) {
         // Set up autonomous command for red alliance.
      } else {
         // This branch will likely never be reached.
      }
      // Set up an autonomous chooser for auton options.
      autoChooser.setDefaultOption("Competition Ready Auton", new OscillateDistanceCommand(m_robotDrive));
      autoChooser.addOption("Drive to Tag 11", new DriveToTagCommand(m_robotDrive, ReefscapeTargetPoses.RED_TAG11_RIGHT));
      autoChooser.addOption("Trajectory Auto", new TrajectoryAutoCommand(m_robotDrive));
      autoChooser.addOption("Forward Tune", new OscillateDistanceCommand(m_robotDrive));
      // (Optional) Add a "Do Nothing" option.
      autoChooser.addOption("No Auto",
            new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive));

      // Configure button bindings.
      configureButtonBindings();

      // Configure default driver command: drive the robot.
      // The default controls for the driver are manual so they can override auton functions.
      m_robotDrive.setDefaultCommand(
            new RunCommand(
                  () -> m_robotDrive.drive(
                        -MathUtil.applyDeadband(m_driverController.getLeftY() * 0.5, OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getLeftX() * 0.5, OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getRightX() * 0.5, OIConstants.kDriveDeadband),
                        true),
                  m_robotDrive));

      // Configure default mech command: control the arm.
      // The default controls for the arm angle and extension are manual so they can override auton functions.
      m_coralArmSubsystem.setDefaultCommand(new ManualCoralArmAdjustCommand(m_coralArmSubsystem, m_mechanismController));
   }

   private void configureButtonBindings() {
      // ************ Driver Controller

      // Create a trigger to cancel any drive commands when joystick inputs exceed a deadband.
      new Trigger(() ->
            Math.abs(m_driverController.getLeftY()) > 0.2 ||
            Math.abs(m_driverController.getLeftX()) > 0.2 ||
            Math.abs(m_driverController.getRightX()) > 0.2)
         .onTrue(new InstantCommand(() -> {
             // Cancel any commands that require the drive subsystem.
             // (If getScheduledCommands() is not available, consider using cancelAll() if acceptable.)
             CommandScheduler.getInstance().cancelAll(); 
         }, m_robotDrive));

      // Right bumper, defensive X-formation.
      new JoystickButton(m_driverController, Button.kR1.value)
         .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

      // Toggle between robot and field oriented.
      dpadDownButton.onTrue(new InstantCommand(m_robotDrive::toggleFieldOriented, m_robotDrive));

      // ************ Mechanism Controller

      SmartDashboard.putString("Target Arm Position", targetArmPosition.toString());
      SmartDashboard.putString("Current Arm Position", currentArmPosition.toString());

      // Right bumper zeros arm encoders.
      new JoystickButton(m_mechanismController, Button.kR1.value)
         .whileTrue(new RunCommand(() -> m_coralArmSubsystem.zeroEncoders(), m_coralArmSubsystem));

      // A button deploys algae arm and starts collector while held.
      new JoystickButton(m_mechanismController, XboxController.Button.kA.value)
         .whileTrue(new RunCommand(() -> m_algaeCollector.moveArm(0.5), m_algaeCollector))
         .whileFalse(new InstantCommand(() -> m_algaeCollector.stopArm(), m_algaeCollector));

      // Y button – algae extender movement.
      new JoystickButton(m_mechanismController, XboxController.Button.kY.value)
         .whileTrue(new RunCommand(() -> m_algaeExtender.moveArm(m_algaeExtender.getInitPos() + 10), m_algaeExtender));

      // Create a trigger to cancel any commands that require the arm subsystem
      // if the mechanism controller's joysticks move outside a deadband.
      new Trigger(() ->
            m_mechanismController.getRightTriggerAxis() > 0.2)
         .onTrue(new InstantCommand(() -> {
             if (m_currentArmCommand != null && m_currentArmCommand.isScheduled()) {
                m_currentArmCommand.cancel();
                m_currentArmCommand = null;
             }
         }, m_coralArmSubsystem));

      // When the operator presses D-pad Up, move to the next higher arm position.
      mech_dpadUpButton.onTrue(new InstantCommand(() -> {
         switch (targetArmPosition) {
            case INIT:
               targetArmPosition = ArmPosition.LOW;
               break;
            case LOW:
               targetArmPosition = ArmPosition.MID;
               break;
            case MID:
               targetArmPosition = ArmPosition.HIGH;
               break;
            case HIGH:
               // Already at highest; optionally wrap around or do nothing.
               break;
         }
         SmartDashboard.putString("Target Arm Position", targetArmPosition.toString());
         SmartDashboard.putString("Current Arm Position", currentArmPosition.toString());
      }, m_coralArmSubsystem));

      // When the operator presses D-pad Down, move to the next lower arm position.
      mech_dpadDownButton.onTrue(new InstantCommand(() -> {
         switch (targetArmPosition) {
            case HIGH:
               targetArmPosition = ArmPosition.MID;
               break;
            case MID:
               targetArmPosition = ArmPosition.LOW;
               break;
            case LOW:
               targetArmPosition = ArmPosition.INIT;
               break;
            case INIT:
               // Already at home; no change.
               break;
         }
         SmartDashboard.putString("Target Arm Position", targetArmPosition.toString());
         SmartDashboard.putString("Current Arm Position", currentArmPosition.toString());
      }, m_coralArmSubsystem));

      new JoystickButton(m_mechanismController, XboxController.Button.kX.value)
      .onTrue(new InstantCommand(() -> {
         if (currentArmPosition != targetArmPosition) {
            currentArmPosition = targetArmPosition;
            if (currentArmPosition == ArmPosition.INIT) {
               m_currentArmCommand = new HomeCoralArmCommand(m_coralArmSubsystem);
            } else {
               m_currentArmCommand = new SetArmPositionCommand(
               m_coralArmSubsystem,
               getTargetAngle(currentArmPosition),
               getTargetExtension(currentArmPosition));
         }}
         m_currentArmCommand.schedule();
      }));
   }

   private double getTargetAngle(ArmPosition pos) {
      switch (pos) {
         case INIT:
            return 0.0;
         case LOW:
            return ArmConstants.lowTgtAngle;
         case MID:
            return ArmConstants.midTgtAngle;
         case HIGH:
            return ArmConstants.highTgtAngle;
         default:
            return 0.0;
      }
   }

   private double getTargetExtension(ArmPosition pos) {
      switch (pos) {
         case INIT:
            return 0.0;
         case LOW:
            return ArmConstants.lowTgtHeight;
         case MID:
            return ArmConstants.midTgtHeight;
         case HIGH:
            return ArmConstants.highTgtHeight;
         default:
            return 0.0;
      }
   }

   /**
    * This method is called by the main Robot class to get the command to run in autonomous.
    */
   public Command getAutonomousCommand() {
      // Return the autonomous command selected from the chooser.
      return autoChooser.getSelected();
   }

   public void periodic() {
      SmartDashboard.putData("Auto Tuning Mode", autoChooser);
   }
}
