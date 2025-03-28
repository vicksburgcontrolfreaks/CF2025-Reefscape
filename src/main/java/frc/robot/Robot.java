
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LedSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  UsbCamera camera1;

  public RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // camera1 = CameraServer.startAutomaticCapture(0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics that you want ran during disabled, autonomous, teleoperated
   * and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /*
     * Runs the Scheduler. This is responsible for polling buttons, adding
     * newly-scheduled commands, running already-scheduled commands, removing
     * finished or interrupted commands, and running subsystem periodic() methods.
     * This must be called from the robot's periodic block in order for anything in
     * the Command-based framework to work.
     */
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    // // Set the Limelight IMU mode to 1 for seeding
    LimelightHelpers.SetIMUMode("limelight", 3);

    LimelightHelpers.PoseEstimate visionEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (visionEstimate != null && visionEstimate.tagCount > 0) {

      Pose2d currentPose = m_robotContainer.getDriveSubsystem().getPose();
      Pose2d visionPose = visionEstimate.pose;

      // Compute the distance between current and vision-derived poses.
      double deltaX = currentPose.getTranslation().getX() - visionPose.getTranslation().getX();
      double deltaY = currentPose.getTranslation().getY() - visionPose.getTranslation().getY();
      double distanceDifference = Math.hypot(deltaX, deltaY);

      // Only reset odometry if the difference is greater than 0.5 meters. Prevents jitter while sitting still.
      if (distanceDifference > 0.5) {
        m_robotContainer.getDriveSubsystem().resetOdometry(visionPose);
      }

      // Also add the vision measurement to fuse with odometry.
      m_robotContainer.getLocalizationSubsystem().poseEstimator.addVisionMeasurement(visionPose,
          visionEstimate.timestampSeconds);
      m_robotContainer.getLedSubsystem().setLEDMode(LedSubsystem.LEDMode.IDLE);

    }

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    LimelightHelpers.SetIMUMode("limelight", 3);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // Stops autonomous when teleop begins; remove to let it run until interrupted.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //TODO FIGURE THIS OUT
    m_robotContainer.getDriveSubsystem().initFieldOrientationForAlliance();

    // LL is seeded with robot orientation, imu inputs are now from LL4 (mode 2)
    LimelightHelpers.SetIMUMode("limelight", 3);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
