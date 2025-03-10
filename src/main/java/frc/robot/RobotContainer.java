//  Control Freaks 2025
// https://chatgpt.com/share/67bd090f-4c08-8005-9e26-2a43e1b26ac6

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutonScoreAndPickup_Red0;
import frc.robot.commands.CollectBallCommand;
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
import frc.robot.commands.TeleopAutoScoreCommand;
import frc.robot.subsystems.NewCoralArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralCollectorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HarpoonSubsystem;
import frc.robot.subsystems.LedSubsystem;
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
   // Determine alliance assignment.
   private final DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

   // Subsystems.
   private final DriveSubsystem m_robotDrive = new DriveSubsystem();
   private final AlgaeArmSubsystem m_algaeArmSubsystem = new AlgaeArmSubsystem();
   private final NewCoralArmSubsystem m_coralArmSubsystem = new NewCoralArmSubsystem();
   private final HarpoonSubsystem m_harpoon = new HarpoonSubsystem();
   private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
   private final LocalizationSubsystem m_localizationSubsystem = new LocalizationSubsystem(m_robotDrive, m_visionSubsystem);
   private final CoralCollectorSubsystem m_CoralCollectorSubsystem = new CoralCollectorSubsystem();
   private final LedSubsystem m_LedSubsystem = new LedSubsystem();

   // Arm position enum.
   public enum ArmPosition {
      INIT, LOW, MID, HIGH;
   }
   private ArmPosition currentArmPosition = ArmPosition.INIT;
   private ArmPosition targetArmPosition = ArmPosition.INIT;

   // Controllers.
   private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
   private final XboxController m_mechanismController = new XboxController(OIConstants.kMechanismControllerPort);
   
   // POV buttons for driver controller.
   private final POVButton dpadUpButton = new POVButton(m_driverController, 0);
   private final POVButton dpadRightButton = new POVButton(m_driverController, 90);
   private final POVButton dpadDownButton = new POVButton(m_driverController, 180);
   private final POVButton dpadLeftButton = new POVButton(m_driverController, 270);
   
   // POV buttons for mechanism controller.
   private final POVButton mech_dpadUpButton = new POVButton(m_mechanismController, 0);
   private final POVButton mech_dpadRightButton = new POVButton(m_mechanismController, 90);
   private final POVButton mech_dpadDownButton = new POVButton(m_mechanismController, 180);
   private final POVButton mech_dpadLeftButton = new POVButton(m_mechanismController, 270);

   // Autonomous chooser.
   public final SendableChooser<Command> autoChooser = new SendableChooser<>();

   // Currently scheduled commands.
   private Command m_currentArmCommand = null;
   private Command m_autoDriveCommand = null;

   public RobotContainer() {
      SmartDashboard.putData("Field", m_localizationSubsystem.getField());

      // Schedule the algae collector initialization command.
      CommandScheduler.getInstance().schedule(
            new InitAlgaeCollectorPositionCommand(m_algaeArmSubsystem, Constants.AlgaeConstants.ready)
      );

      // Set up autonomous chooser options.
      autoChooser.setDefaultOption("Auton Score L1",
            new InitializeLocalizationCommand(m_robotDrive, m_localizationSubsystem)
                  .andThen(new DynamicDriveToTagCommand(m_robotDrive, m_localizationSubsystem, true))
                  .andThen(new InitAlgaeCollectorPositionCommand(m_algaeArmSubsystem, Constants.AlgaeConstants.ready))
                  .andThen(new RunAlgaeCollectorWheelsCommand(m_algaeArmSubsystem, 0.5, 1.0))
      );
      autoChooser.setDefaultOption("Score, Pickup, and Score",
            new AutonScoreAndPickup_Red0(m_robotDrive, m_localizationSubsystem, m_visionSubsystem, m_coralArmSubsystem)
      );
      autoChooser.addOption("No Auto",
            new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive)
      );

      // Configure button bindings.
      configureButtonBindings();

      // Set default driver command with exponential scaling.
      m_robotDrive.setDefaultCommand(new RunCommand(() -> {
         double forward = expoScale(-m_driverController.getLeftY(), 1.5);
         double strafe = expoScale(-m_driverController.getLeftX(), 3);
         double rotation = expoScale(-m_driverController.getRightX(), 2);
         m_robotDrive.drive(forward, strafe, rotation, true);
      }, m_robotDrive));

      // Set default mechanism command.
      m_coralArmSubsystem.setDefaultCommand(
            new ManualCoralArmAdjustCommand(m_coralArmSubsystem, m_mechanismController)
      );
   }

   private void configureButtonBindings() {
      // --- Driver Controller Bindings ---
      // Left bumper reduces drive speed.
      new JoystickButton(m_driverController, Button.kL1.value)
            .whileTrue(new InstantCommand(() -> m_robotDrive.setSpeedMultiplier(0.1), m_robotDrive))
            .whileFalse(new InstantCommand(() -> m_robotDrive.setSpeedMultiplier(1.0), m_robotDrive));

      // Right bumper resets field orientation.
      new JoystickButton(m_driverController, Button.kR1.value)
            .onTrue(new InstantCommand(() -> m_robotDrive.resetFieldOrientation(), m_robotDrive));

      // Trigger: cancel drive commands when joystick inputs exceed deadband.
      new Trigger(() -> Math.abs(m_driverController.getLeftY()) > 0.2 ||
            Math.abs(m_driverController.getLeftX()) > 0.2 ||
            Math.abs(m_driverController.getRightX()) > 0.2)
            .onTrue(new InstantCommand(() -> {
               if (m_autoDriveCommand != null && m_autoDriveCommand.isScheduled()) {
                  m_autoDriveCommand.cancel();
               }
            }, m_robotDrive));

      // Right bumper also sets defensive X-formation.
      new JoystickButton(m_driverController, Button.kR1.value)
            .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

      // DPad Down toggles field-oriented drive.
      dpadDownButton.onTrue(new InstantCommand(m_robotDrive::toggleFieldOriented, m_robotDrive));

      // Teleop Auto Score: Bind Xbox button X for one scoring mode.
      new JoystickButton(m_driverController, XboxController.Button.kX.value)
            .onTrue(new InstantCommand(() -> {
               m_autoDriveCommand = new TeleopAutoScoreCommand(m_robotDrive, m_localizationSubsystem, m_visionSubsystem,
                     m_coralArmSubsystem, false);
               m_autoDriveCommand.schedule();
            }));
      // (Optionally, add a second binding for the other scoring mode if needed.)
      
      // Right Trigger starts ball collection.
      new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.2)
            .onTrue(new CollectBallCommand(m_algaeArmSubsystem));
      
      // Left Trigger shoots ball and retracts ball collector.
      new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2)
            .onTrue(new ReleaseBallCommand(m_algaeArmSubsystem));

      // --- Mechanism Controller Bindings ---
      SmartDashboard.putString("Target Arm Position", targetArmPosition.toString());
      SmartDashboard.putString("Current Arm Position", currentArmPosition.toString());

      // Right bumper on mechanism controller zeros arm encoders.
      new JoystickButton(m_mechanismController, XboxController.Button.kRightBumper.value)
            .onTrue(new RunCommand(() -> {
                m_coralArmSubsystem.zeroEncoders();
                m_algaeArmSubsystem.zeroEncoders();
                m_CoralCollectorSubsystem.zeroEncoders();
            }, m_coralArmSubsystem));

      // Harpoon control: DPad Right for forward, DPad Left for reverse.
      mech_dpadRightButton.whileTrue(new RunCommand(() -> m_harpoon.setMotor(0.5), m_harpoon))
            .whileFalse(new InstantCommand(() -> m_harpoon.stop(), m_harpoon));
      mech_dpadLeftButton.whileTrue(new RunCommand(() -> m_harpoon.setMotor(-0.5), m_harpoon))
            .whileFalse(new InstantCommand(() -> m_harpoon.stop(), m_harpoon));

      // Cancel arm commands if mechanism controller joysticks move beyond deadband.
      new Trigger(() -> Math.abs(m_mechanismController.getLeftY()) > 0.2 ||
            Math.abs(m_mechanismController.getRightY()) > 0.2)
            .onTrue(new InstantCommand(() -> {
               if (m_currentArmCommand != null && m_currentArmCommand.isScheduled()) {
                  m_currentArmCommand.cancel();
                  m_currentArmCommand = null;
               }
            }, m_coralArmSubsystem));

      // DPad Up on mechanism controller: increment arm position.
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

      // DPad Down on mechanism controller: decrement arm position.
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

      // X Button on mechanism controller: execute the selected arm command.
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
                           getTargetExtension(currentArmPosition)
                     );
                  }
               }
               m_currentArmCommand.schedule();
            }));

      // B Button on mechanism controller: execute the coral scoring sequence.
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
                     // do nothing for INIT or unspecified state.
                     break;
               }
            }, m_coralArmSubsystem));
   }

   // Scales input exponentially.
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
    * Returns the autonomous command selected from the chooser.
    */
   public Command getAutonomousCommand() {
      return autoChooser.getSelected();
   }

   public void periodic() {
      SmartDashboard.putData("Auto Tuning Mode", autoChooser);
      
      // LED control based on robot state.
      if (!DriverStation.isEnabled()) {
         if (m_visionSubsystem.getDetectedTagIDFromNT() > 1) {
            m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.SOLID_GREEN);
         } else {
            m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.FLASH_GREEN);
         }
      } else {
         switch (currentArmPosition) {
            case HIGH:
               m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.ARM_HIGH);
               break;
            case MID:
               m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.ARM_MID);
               break;
            case LOW:
               m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.ARM_LOW);
               break;
            default:
               m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.IDLE);
               break;
         }
         if (DriverStation.getMatchTime() < 15) {
            m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.MATCH_END_FLASH);
         }
      }
   }
}
