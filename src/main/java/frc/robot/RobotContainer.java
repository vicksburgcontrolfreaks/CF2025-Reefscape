// File: RobotContainer.java
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.AutonSelector;
import frc.robot.autonomous.AutonScoreAndPickup_Blue0;
import frc.robot.autonomous.AutonScoreAndPickup_Blue1;
import frc.robot.autonomous.AutonScoreAndPickup_Red0;
import frc.robot.autonomous.AutonScoreAndPickup_Red1;
import frc.robot.autonomous.AutonScore_BlueCenter;
import frc.robot.autonomous.AutonScore_RedCenter;
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
import frc.robot.commands.SetupHangCommand;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
   private final LocalizationSubsystem m_localizationSubsystem = new LocalizationSubsystem(m_robotDrive,
         m_visionSubsystem);
   private final CoralCollectorSubsystem m_CoralCollectorSubsystem = new CoralCollectorSubsystem();
   private final LedSubsystem m_LedSubsystem = new LedSubsystem();
   private final AutonSelector autoSelector = new AutonSelector(
         m_robotDrive,
         m_localizationSubsystem,
         m_visionSubsystem,
         m_coralArmSubsystem,
         m_algaeArmSubsystem);

   // Arm position enum.
   public enum ArmPosition {
      INIT, LOW, MID, HIGH;
   }

   // You can still use global variables if needed…
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
      // Optionally add field display here.
      // SmartDashboard.putData("Field", m_localizationSubsystem.getField());

      // Schedule the algae collector initialization command.
      CommandScheduler.getInstance().schedule(
            new InitAlgaeCollectorPositionCommand(m_algaeArmSubsystem, Constants.AlgaeConstants.ready));

      // // Set up autonomous chooser options.
      // autoChooser.setDefaultOption("Red 0",
      // new AutonScoreAndPickup_Red0(m_robotDrive, m_localizationSubsystem,
      // m_visionSubsystem,
      // m_coralArmSubsystem, m_algaeArmSubsystem));
      // autoChooser.addOption("Red 1",
      // new AutonScoreAndPickup_Red1(m_robotDrive, m_localizationSubsystem,
      // m_visionSubsystem,
      // m_coralArmSubsystem, m_algaeArmSubsystem));
      // autoChooser.addOption("Red Center",
      // new AutonScore_RedCenter(m_robotDrive, m_localizationSubsystem,
      // m_visionSubsystem,
      // m_coralArmSubsystem, m_algaeArmSubsystem));
      // autoChooser.addOption("Blue 0",
      // new AutonScoreAndPickup_Blue0(m_robotDrive, m_localizationSubsystem,
      // m_visionSubsystem,
      // m_coralArmSubsystem, m_algaeArmSubsystem));
      // autoChooser.addOption("Blue 1",
      // new AutonScoreAndPickup_Blue1(m_robotDrive, m_localizationSubsystem,
      // m_visionSubsystem,
      // m_coralArmSubsystem, m_algaeArmSubsystem));
      // autoChooser.addOption("Blue Center",
      // new AutonScore_BlueCenter(m_robotDrive, m_localizationSubsystem,
      // m_visionSubsystem,
      // m_coralArmSubsystem, m_algaeArmSubsystem));
      // autoChooser.addOption("No Auto",
      // new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive));

      // Configure button bindings.
      configureButtonBindings();

      // Set default driver command with exponential scaling.
      m_robotDrive.setDefaultCommand(new RunCommand(() -> {
         double forward = expoScale(-m_driverController.getLeftY() * 0.75, 2);
         double strafe = expoScale(-m_driverController.getLeftX() * 0.75, 2);
         double rotation = expoScale(-m_driverController.getRightX(), 2);
         m_robotDrive.drive(forward, strafe, rotation, true);
      }, m_robotDrive));

      // Set default driver command with two factor linear scaling.
      m_robotDrive.setDefaultCommand(new RunCommand(() -> {
         double forward = scaleDriveInput(-m_driverController.getLeftY());
         double strafe = scaleDriveInput(-m_driverController.getLeftX());
         double rotation = scaleDriveInput(-m_driverController.getRightX());
         m_robotDrive.drive(forward, strafe, rotation, true);
      }, m_robotDrive));

      // Set default mechanism command.
      m_coralArmSubsystem.setDefaultCommand(
            new ManualCoralArmAdjustCommand(m_coralArmSubsystem, m_mechanismController));
   }

   private void configureButtonBindings() {
      // --- Driver Controller Bindings ---
      new JoystickButton(m_driverController, Button.kL1.value)
            .whileTrue(new InstantCommand(() -> m_robotDrive.setSpeedMultiplier(0.1), m_robotDrive))
            .whileFalse(new InstantCommand(() -> m_robotDrive.setSpeedMultiplier(1.0), m_robotDrive));

      //Right bumper resets field orientation and sets wheel in x position
      new JoystickButton(m_driverController, Button.kR1.value)
            .onTrue(new InstantCommand(() -> m_robotDrive.resetFieldOrientation(), m_robotDrive))
            .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

      //Cancels any auto driving command when there is input from the joysticks
      new Trigger(() -> Math.abs(m_driverController.getLeftY()) > 0.2 ||
            Math.abs(m_driverController.getLeftX()) > 0.2 ||
            Math.abs(m_driverController.getRightX()) > 0.2)
            .onTrue(new InstantCommand(() -> {
               if (m_autoDriveCommand != null && m_autoDriveCommand.isScheduled()) {
                  m_autoDriveCommand.cancel();
               }
            }, m_robotDrive));

      // new JoystickButton(m_driverController, Button.kR1.value)
      //       .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

      dpadDownButton.onTrue(new InstantCommand(m_robotDrive::toggleFieldOriented, m_robotDrive));

      // Teleop auto scoring commands: Drive to coral branch and deploy arm
      // Right
      new JoystickButton(m_driverController, XboxController.Button.kB.value)
            .onTrue(new InstantCommand(() -> {
               m_autoDriveCommand = new TeleopAutoScoreCommand(m_robotDrive, m_localizationSubsystem, m_visionSubsystem,
                     m_coralArmSubsystem, false);
               m_autoDriveCommand.schedule();
            }));
      // Left
      new JoystickButton(m_driverController, XboxController.Button.kX.value)
            .onTrue(new InstantCommand(() -> {
               m_autoDriveCommand = new TeleopAutoScoreCommand(m_robotDrive, m_localizationSubsystem, m_visionSubsystem,
                     m_coralArmSubsystem, true);
               m_autoDriveCommand.schedule();
            }));
      //Right trigger collects algae ball
      new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.2)
            .onTrue(new CollectBallCommand(m_algaeArmSubsystem));
      //Left trigger releases ball and stows the algae arm
      new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2)
            .onTrue(new ReleaseBallCommand(m_algaeArmSubsystem));

      // --- Mechanism Controller Bindings ---
      //Right bumper zeros arm encoders
      new JoystickButton(m_mechanismController, XboxController.Button.kRightBumper.value)
            .onTrue(new RunCommand(() -> {
               m_coralArmSubsystem.zeroEncoders();
               m_algaeArmSubsystem.zeroEncoders();
            }, m_coralArmSubsystem, m_algaeArmSubsystem));

      mech_dpadRightButton.whileTrue(new RunCommand(() -> m_harpoon.setMotor(0.5), m_harpoon))
            .whileFalse(new InstantCommand(() -> m_harpoon.stop(), m_harpoon));
      mech_dpadLeftButton.whileTrue(new RunCommand(() -> m_harpoon.setMotor(-0.5), m_harpoon))
            .whileFalse(new InstantCommand(() -> m_harpoon.stop(), m_harpoon));

      new Trigger(() -> Math.abs(m_mechanismController.getLeftY()) > 0.2 ||
            Math.abs(m_mechanismController.getRightY()) > 0.2)
            .onTrue(new InstantCommand(() -> {
               if (m_currentArmCommand != null && m_currentArmCommand.isScheduled()) {
                  m_currentArmCommand.cancel();
                  m_currentArmCommand = null;
               }
            }, m_coralArmSubsystem));

      mech_dpadUpButton.onTrue(new InstantCommand(() -> {
         switch (ArmConstants.targetArmPosition) {
            case INIT:
               ArmConstants.targetArmPosition = ArmConstants.ArmPosition.LOW;
               m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.ARM_LOW);
               break;
            case LOW:
               ArmConstants.targetArmPosition = ArmConstants.ArmPosition.MID;
               m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.ARM_MID);
               break;
            case MID:
               ArmConstants.targetArmPosition = ArmConstants.ArmPosition.HIGH;
               m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.ARM_HIGH);
               break;
            case HIGH:
               // do nothing
               break;
         }
      }, m_coralArmSubsystem));

      mech_dpadDownButton.onTrue(new InstantCommand(() -> {
         switch (ArmConstants.targetArmPosition) {
            case HIGH:
               ArmConstants.targetArmPosition = ArmConstants.ArmPosition.MID;
               m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.ARM_MID);
               break;
            case MID:
               ArmConstants.targetArmPosition = ArmConstants.ArmPosition.LOW;
               m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.ARM_LOW);
               break;
            case LOW:
               ArmConstants.targetArmPosition = ArmConstants.ArmPosition.INIT;
               break;
            case INIT:
               // do nothing
               break;
         }
      }, m_coralArmSubsystem));

      new JoystickButton(m_mechanismController, XboxController.Button.kX.value)
            .onTrue(new InstantCommand(() -> {
               if (ArmConstants.currentArmPosition != ArmConstants.targetArmPosition) {
                  ArmConstants.currentArmPosition = ArmConstants.targetArmPosition;
                  if (ArmConstants.currentArmPosition == ArmConstants.ArmPosition.INIT) {
                     m_currentArmCommand = new HomeCoralArmCommand(m_coralArmSubsystem);
                  } else {
                     m_currentArmCommand = new SetArmPositionCommand(
                           m_coralArmSubsystem,
                           getTargetAngle(ArmConstants.currentArmPosition),
                           getTargetExtension(ArmConstants.currentArmPosition));
                  }
               }
               m_currentArmCommand.schedule();
            }));

      new JoystickButton(m_mechanismController, XboxController.Button.kB.value)
            .onTrue(new InstantCommand(() -> {
               switch (ArmConstants.currentArmPosition) {
                  case LOW:
                     new LowScoringSequenceCommand(m_coralArmSubsystem).schedule();
                     new WaitCommand(0.1);
                     ArmConstants.currentArmPosition = ArmConstants.ArmPosition.INIT;
                     break;
                  case MID:
                     new MidScoringSequenceCommand(m_coralArmSubsystem).schedule();
                     new WaitCommand(0.1);
                     ArmConstants.currentArmPosition = ArmConstants.ArmPosition.INIT;
                     break;
                  case HIGH:
                     new HighScoringSequenceCommand(m_coralArmSubsystem).schedule();
                     new WaitCommand(0.1);
                     ArmConstants.currentArmPosition = ArmConstants.ArmPosition.INIT;
                     break;
                  default:
                     break;
               }
            }, m_coralArmSubsystem));
   }

   public double scaleDriveInput(double input) {
      // Get absolute value and the sign.
      double sign = Math.signum(input);
      double absInput = Math.abs(input);
      double output;
      
      if (absInput <= 0.5) {
          // Fine control: scale by 0.5
          output = 0.5 * absInput;
      } else {
          // Above 0.5: add additional multiplier (here, 1.5)
          output = 0.25 + 1.5 * (absInput - 0.5);
      }
      
      // Return with the original sign.
      return sign * output;
  }
  

   public double expoScale(double input, double exponent) {
      return Math.copySign(Math.pow(Math.abs(input), exponent), input);
   }

   private double getTargetAngle(ArmConstants.ArmPosition pos) {
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

   private double getTargetExtension(ArmConstants.ArmPosition pos) {
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

   public Command getAutonomousCommand() {
      return autoSelector.selectAutoCommand();
   }

   public DriveSubsystem getDriveSubsystem() {
      return m_robotDrive;
   }

   public VisionSubsystem getVisionSubsystem() {
      return m_visionSubsystem;
   }

   public LocalizationSubsystem getLocalizationSubsystem() {
      return m_localizationSubsystem;
   }

   public LedSubsystem getLedSubsystem() {
      return m_LedSubsystem;
   }

   // Centralize SmartDashboard updates here.
   public void periodic() {
      if (Constants.COMP_CODE) {
         SmartDashboard.putData("Auto Tuning Mode", autoChooser);
         SmartDashboard.putString("Target Arm Position", ArmConstants.targetArmPosition.toString());
         SmartDashboard.putString("Current Arm Position", ArmConstants.currentArmPosition.toString());
      } else {
         // put additional debugging calls here
      }

      // LED control based on robot state.
      if (!DriverStation.isEnabled()) {
         if (m_visionSubsystem.getDetectedTagIDFromNT() > 0) {
            m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.IDLE);
         } else {
            m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.IDLE);
         }
      } else {
         // You can update LED mode here if needed.
         if (DriverStation.getMatchTime() < 15) {
            m_LedSubsystem.setLEDMode(LedSubsystem.LEDMode.MATCH_END_FLASH);
         }
      }
   }
}
