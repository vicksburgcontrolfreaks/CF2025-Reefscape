//  Control Freaks 2025

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.ReefscapeTargetPoses;
import frc.robot.autonomous.OscillateDistanceCommand;
import frc.robot.autonomous.TrajectoryAutoCommand;
import frc.robot.commands.HomeCoralArmCommand;
import frc.robot.commands.ManualCoralArmAdjustCommand;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ScoreCoralParallelCommand;
import frc.robot.commands.SetCoralArmPositionCommand;
import frc.robot.commands.RaiseCoralArm;
import frc.robot.commands.ReleaseCoralCmd;

import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlgaeCollectorSubsystem;
import frc.robot.subsystems.AlgaeExtenderSubsystem;
import frc.robot.subsystems.HarpoonSubsystem;

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
   private final HarpoonSubsystem m_harpoon = new HarpoonSubsystem();
   private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
   private final LocalizationSubsystem m_localizationSubsystem = new LocalizationSubsystem(m_robotDrive);
   private final CoralArmSubsystem m_coralArmSubsystem = new CoralArmSubsystem();

   // Variable to tracking the scoring position (INIT, LOW, LED, HIGH)
   private int m_scoringPosition = ArmConstants.TGT_HIGH; // Default scoring position: High
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

   public RobotContainer() {
      // Determine alliance assignment
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

      // The default controls for the arm angle and extension are manual so they can
      // override auton functions
      m_coralArmSubsystem
            .setDefaultCommand(new ManualCoralArmAdjustCommand(m_coralArmSubsystem, m_mechanismController));

   }

   private void configureButtonBindings() {
      // ************ Driver controller
      // Right bumper, defensive X-formation
      new JoystickButton(m_driverController, Button.kR1.value)
            .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

      // Use ScoreCoralParallelCommand with the scoring side flag: true = left, false
      // = right.
      dpadLeftButton.onTrue(new ScoreCoralParallelCommand(m_robotDrive, m_visionSubsystem, m_coralArmSubsystem, true));
      dpadRightButton
            .onTrue(new ScoreCoralParallelCommand(m_robotDrive, m_visionSubsystem, m_coralArmSubsystem, false));

      // toggle between robot and field oriented
      dpadDownButton.onTrue(new InstantCommand(m_robotDrive::toggleFieldOriented, m_robotDrive));

      // ************ Mech controller
      // A button deploys algae arm and starts collector while held
      new JoystickButton(m_mechanismController, XboxController.Button.kA.value)
            .whileTrue(new RunCommand(() -> {
               m_algaeCollector.moveArm(0.5);
            }, m_algaeCollector)) // Start the collector
            .whileFalse(new InstantCommand(() -> m_algaeCollector.stopArm(), m_algaeCollector)); // Stop when released

      // X button
      new JoystickButton(m_mechanismController, XboxController.Button.kX.value)
            .whileTrue(new RunCommand(() -> {
               m_algaeExtender.moveArm(m_algaeExtender.getInitPos() + 10);
            }, m_algaeExtender));

      // Y button
      new JoystickButton(m_mechanismController, XboxController.Button.kY.value)
            .whileTrue(new RunCommand(() -> {
               m_algaeExtender.moveArm(m_algaeExtender.getInitPos());
            }, m_algaeExtender));

      // Coral Arm
      // D-pad up increments scoring position if not already at high.
      mech_dpadUpButton.onTrue(
            new InstantCommand(() -> {
               if (m_scoringPosition < Constants.ArmConstants.TGT_HIGH) {
                  m_scoringPosition++;
                  System.out.println("Scoring position increased to: " + m_scoringPosition);
               }
            }, m_coralArmSubsystem)
                  .andThen(new SetCoralArmPositionCommand(
                        m_coralArmSubsystem,
                        getTargetAngle(m_scoringPosition),
                        getTargetExtension(m_scoringPosition))));

      // D-pad down decrements scoring position if not already stowed.
      mech_dpadDownButton.onTrue(
            new InstantCommand(() -> {
               if (m_scoringPosition > Constants.ArmConstants.TGT_INIT) {
                  m_scoringPosition--;
                  System.out.println("Scoring position decreased to: " + m_scoringPosition);
               }
            }, m_coralArmSubsystem)
                  .andThen(new SetCoralArmPositionCommand(
                        m_coralArmSubsystem,
                        getTargetAngle(m_scoringPosition),
                        getTargetExtension(m_scoringPosition))));

      // D-pad right/left to toggle scoring side.
      mech_dpadRightButton.onTrue(new InstantCommand(() -> {
         m_scoringSideLeft = false;
         System.out.println("Scoring side set to RIGHT");
      }));
      mech_dpadLeftButton.onTrue(new InstantCommand(() -> {
         m_scoringSideLeft = true;
         System.out.println("Scoring side set to LEFT");
      }));

      // Left Stick Y controls manual angle of arm
      // Right Stick Y controls manual extension of arm
      // Control speed here: commands.ManualCoralArmAdjustCommand.java

   }

   /**
    * This method is called by the main Robot class to get the command to run in
    * autonomous.
    */
   public Command getAutonomousCommand() {
      // Return the autonomous command selected from the chooser.
      return autoChooser.getSelected();
   }

   // These two methods convert the enumerated target position into an angle and
   // extension value
   private double getTargetAngle(int scoringPosition) {
      switch (scoringPosition) {
         case Constants.ArmConstants.TGT_LOW:
            return Constants.ArmConstants.lowTgtAngle;
         case Constants.ArmConstants.TGT_MID:
            return Constants.ArmConstants.midTgtAngle;
         case Constants.ArmConstants.TGT_HIGH:
            return Constants.ArmConstants.highTgtAngle;
         default: // HOME or invalid, default to 0
            return 0.0;
      }
   }

   private double getTargetExtension(int scoringPosition) {
      switch (scoringPosition) {
         case Constants.ArmConstants.TGT_LOW:
            return Constants.ArmConstants.lowTgtHeight;
         case Constants.ArmConstants.TGT_MID:
            return Constants.ArmConstants.midTgtHeight;
         case Constants.ArmConstants.TGT_HIGH:
            return Constants.ArmConstants.highTgtHeight;
         default: // HOME or invalid, default to 0
            return 0.0;
      }
   }

}
