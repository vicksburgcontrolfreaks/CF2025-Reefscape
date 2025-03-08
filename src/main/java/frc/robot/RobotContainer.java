//  Control Freaks 2025
//https://chatgpt.com/share/67bd090f-4c08-8005-9e26-2a43e1b26ac6

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.TrajectoryAutoCommand;
import frc.robot.commands.CollectBallCommand;
import frc.robot.commands.DeployAndStowAlgaeArmCommand;
import frc.robot.commands.ReleaseBallCommand;
import frc.robot.commands.RunAlgaeCollectorWheelsCommand;
import frc.robot.commands.DynamicDriveToTagCommand;
import frc.robot.commands.HighScoringSequenceCommand;
import frc.robot.commands.HomeCoralArmCommand;
import frc.robot.commands.InitAlgaeCollectorPositionCommand;
import frc.robot.commands.InitializeLocalizationCommand;
import frc.robot.commands.LowScoringSequenceCommand;
import frc.robot.commands.ManualCoralArmAdjustCommand;
import frc.robot.commands.MidScoringSequenceCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.subsystems.NewCoralArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralCollectorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
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
   private final AlgaeArmSubsystem m_algaeArmSubsystem = new AlgaeArmSubsystem();
   private final NewCoralArmSubsystem m_coralArmSubsystem = new NewCoralArmSubsystem();
   private final HarpoonSubsystem m_harpoon = new HarpoonSubsystem();
   private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
   private final LocalizationSubsystem m_localizationSubsystem = new LocalizationSubsystem(m_robotDrive,
         m_visionSubsystem);
   private final CoralCollectorSubsystem m_CoralCollectorSubsystem = new CoralCollectorSubsystem();

   // Use the enum for arm positions.
   public enum ArmPosition {
      INIT, LOW, MID, HIGH;
   }

   private ArmPosition currentArmPosition = ArmPosition.INIT;
   private ArmPosition targetArmPosition = ArmPosition.INIT;

   // Variable to track scoring side (true = left, false = right); default left.
   // private boolean m_scoringSideLeft = true;

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
   // Field to store the currently scheduled drive command.
   // m_autoDriveCommand = null;
   // m_autoDriveCommand.schedule();

   public RobotContainer() {
      
      SmartDashboard.putData("Field", m_localizationSubsystem.getField());

      // Set the algae arm to the "ready position" to get it out of the way a bit
      CommandScheduler.getInstance()
            .schedule(new InitAlgaeCollectorPositionCommand(m_algaeArmSubsystem, Constants.AlgaeConstants.ready));

      // Set up an autonomous chooser for auton options.
      autoChooser.setDefaultOption("Competition Ready Auton",
            new DynamicDriveToTagCommand(m_robotDrive, m_localizationSubsystem));
      // autoChooser.setDefaultOption("Competition Ready Auton",
      // new DynamicDriveToTagCommand(m_robotDrive, m_localizationSubsystem));
      autoChooser.addOption("Trajectory Auto", new TrajectoryAutoCommand(m_robotDrive));
      autoChooser.addOption(
            "Dynamic Drive to Tag with Init",
            new InitializeLocalizationCommand(m_robotDrive, m_localizationSubsystem)
                  .andThen(new DynamicDriveToTagCommand(m_robotDrive, m_localizationSubsystem))
                  .andThen(new InitAlgaeCollectorPositionCommand(m_algaeArmSubsystem, Constants.AlgaeConstants.ready))
                  .andThen(new RunAlgaeCollectorWheelsCommand(m_algaeArmSubsystem, 0.5, 1.0)));
      autoChooser.addOption(
            "Trajectory Auto Score",
            new TrajectoryAutoCommand(m_robotDrive)
                  .andThen(new DeployAndStowAlgaeArmCommand(m_algaeArmSubsystem)));
      // (Optional) Add a "Do Nothing" option.
      autoChooser.addOption("No Auto",
            new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive));

      // Configure button bindings.
      configureButtonBindings();

      // Configure default driver command: drive the robot.
      // The default controls for the driver are manual so they can override auton
      // functions.
      // m_robotDrive.setDefaultCommand(
      // new RunCommand(
      // () -> m_robotDrive.drive(
      // -scaleJoystick(m_driverController.getLeftY()),
      // -scaleJoystick(m_driverController.getLeftX()),
      // -scaleJoystick(m_driverController.getRightX()),
      // true),
      // m_robotDrive));
      // exponetial drive controls
      m_robotDrive.setDefaultCommand(
            new RunCommand(() -> {
               double forward = expoScale(-m_driverController.getLeftY(), 1.5); // example exponent 2
               double strafe = expoScale(-m_driverController.getLeftX(), 3);
               double rotation = expoScale(-m_driverController.getRightX(), 2);
               m_robotDrive.drive(forward, strafe, rotation, true);
            }, m_robotDrive));

      // Configure default mech command: control the arm.
      // The default controls for the arm angle and extension are manual so they can
      // override auton functions.
      m_coralArmSubsystem
            .setDefaultCommand(new ManualCoralArmAdjustCommand(m_coralArmSubsystem, m_mechanismController));
   }

   private void configureButtonBindings() {
      // ************ Driver Controller
      // left bumper reduces speed
      new JoystickButton(m_driverController, Button.kL1.value)
            .whileTrue(new InstantCommand(() -> m_robotDrive.setSpeedMultiplier(0.1), m_robotDrive))
            .whileFalse(new InstantCommand(() -> m_robotDrive.setSpeedMultiplier(1.0), m_robotDrive));

      // right bumper reset field orientation
      new JoystickButton(m_driverController, Button.kR1.value)
            .onTrue(new InstantCommand(() -> m_robotDrive.resetFieldOrientation(), m_robotDrive));

      // Leave this here for now. We will eventually want to add a trigger to cancel
      // auton driving commands
      // // Create a trigger to cancel any drive commands when joystick inputs exceed
      // a
      // // deadband.
      // new Trigger(() ->
      // Math.abs(m_driverController.getLeftY()) > 0.2 ||
      // Math.abs(m_driverController.getLeftX()) > 0.2 ||
      // Math.abs(m_driverController.getRightX()) > 0.2)
      // .onTrue(new InstantCommand(() -> {
      // if (m_autoDriveCommand != null && m_autoDriveCommand.isScheduled()) {
      // m_autoDriveCommand.cancel();
      // }
      // }, m_robotDrive));

      // Right bumper, defensive X-formation.
      new JoystickButton(m_driverController, Button.kR1.value)
            .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

      // Toggle between robot and field oriented.
      dpadDownButton.onTrue(new InstantCommand(m_robotDrive::toggleFieldOriented, m_robotDrive));

      // DPad Right drives to right scoring position of reef section visible to the
      // robot
      // dpadRightButton.onTrue(new ScoreCoralDriveCommand(m_robotDrive,
      // m_visionSubsystem, false));

      // DPad Left drives to right scoring position of reef section visible to the
      // robot
      // dpadLeftButton.onTrue(new ScoreCoralDriveCommand(m_robotDrive,
      // m_visionSubsystem, true));

      new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.2)
            .onTrue(new CollectBallCommand(m_algaeArmSubsystem));

      new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2)
            .onTrue(new ReleaseBallCommand(m_algaeArmSubsystem));

      // ************ Mechanism Controller

      SmartDashboard.putString("Target Arm Position", targetArmPosition.toString());
      SmartDashboard.putString("Current Arm Position", currentArmPosition.toString());

      // Right bumper zeros arm encoders.
      new JoystickButton(m_mechanismController, XboxController.Button.kRightBumper.value)
            .onTrue(new RunCommand(() -> m_coralArmSubsystem.zeroEncoders(), m_coralArmSubsystem))
            .onTrue(new RunCommand(() -> m_algaeArmSubsystem.zeroEncoders(), m_algaeArmSubsystem))
            .onTrue(new RunCommand(() -> m_CoralCollectorSubsystem.zeroEncoders(), m_CoralCollectorSubsystem));

      // new JoystickButton(m_mechanismController, XboxController.Button.kB.value)
      // .onTrue(new ScoreCoralArmCommand(m_coralArmSubsystem));

      // new Trigger(() -> m_mechanismController.getLeftTriggerAxis() > 0.2)
      // .onTrue(new CollectBallCommand(m_algaeArmSubsystem));

      // new Trigger(() -> m_mechanismController.getRightTriggerAxis() > 0.2)
      // .onTrue(new ReleaseBallCommand(m_algaeArmSubsystem));

      mech_dpadRightButton
            .whileTrue(new RunCommand(() -> m_harpoon.setMotor(0.5), m_harpoon))
            .whileFalse(new InstantCommand(() -> m_harpoon.stop(), m_harpoon));

      mech_dpadLeftButton
            .whileTrue(new RunCommand(() -> m_harpoon.setMotor(-0.5), m_harpoon))
            .whileFalse(new InstantCommand(() -> m_harpoon.stop(), m_harpoon));

      // Create a trigger to cancel any commands that require the arm subsystem
      // if the mechanism controller's joysticks move outside a deadband.
      new Trigger(() -> Math.abs(m_mechanismController.getLeftY()) > 0.2 ||
            Math.abs(m_mechanismController.getRightY()) > 0.2)
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
               // do nothing
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
               // do nothing
               break;
         }
         SmartDashboard.putString("Target Arm Position", targetArmPosition.toString());
         SmartDashboard.putString("Current Arm Position", currentArmPosition.toString());
      }, m_coralArmSubsystem));

      // X Button executes the selected arm position set by DPAD Up/Down
      new JoystickButton(m_mechanismController, XboxController.Button.kX.value)
            .onTrue(new InstantCommand(() -> {
               if (currentArmPosition != targetArmPosition) {
                  currentArmPosition = targetArmPosition;
                  if (currentArmPosition == ArmPosition.INIT) {
                     m_currentArmCommand = new HomeCoralArmCommand(m_coralArmSubsystem);
                     // new SetCoralCollectorPositionCommand(m_CoralCollectorSubsystem,
                     // 0).schedule();

                  } else {
                     m_currentArmCommand = new SetArmPositionCommand(
                           m_coralArmSubsystem,
                           getTargetAngle(currentArmPosition),
                           getTargetExtension(currentArmPosition));
                  }
               }
               m_currentArmCommand.schedule();
            }));

      // B Button executes the Coral Scoring Sequence based on the arm scoring
      // position
      new JoystickButton(m_mechanismController, XboxController.Button.kB.value)
            .onTrue(new InstantCommand(() -> {
               switch (currentArmPosition) {
                  case LOW:
                     new LowScoringSequenceCommand(m_coralArmSubsystem).schedule();
                     break;
                  case MID:
                     new MidScoringSequenceCommand(m_coralArmSubsystem).schedule();
                     break;
                  case HIGH:
                     new HighScoringSequenceCommand(m_coralArmSubsystem).schedule();
                     break;
                  default:
                     // For INIT or any unspecified state, you could do nothing or a default action.
                     break;
               }
            }, m_coralArmSubsystem));

   }

   // This method scales input from the driver joysticks
   public double expoScale(double input, double exponent) {
      return Math.copySign(Math.pow(Math.abs(input), exponent), input);
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
    * This method is called by the main Robot class to get the command to run in
    * autonomous.
    */
   public Command getAutonomousCommand() {
      // Return the autonomous command selected from the chooser.
      return autoChooser.getSelected();
      // return new DriveForwardOneMeterCommand(m_robotDrive);
   }

   public void periodic() {
      SmartDashboard.putData("Auto Tuning Mode", autoChooser);
   }
}
