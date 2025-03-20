// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.CurrentMonitor;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // Field oriented flag; true = field-oriented, false = robot-oriented.
  private boolean m_fieldOriented = true;

  // Method to toggle the drive mode.
  public void toggleFieldOriented() {
    m_fieldOriented = !m_fieldOriented;
  }

  // Getter for the drive mode.
  public boolean isFieldOriented() {
    return m_fieldOriented;
  }

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  // Instantiate the built-in accelerometer.
  private final BuiltInAccelerometer m_accel = new BuiltInAccelerometer();

  /**
   * Checks for a collision based on accelerometer readings.
   * This method returns true if the acceleration in any axis exceeds the
   * threshold (in g's).
   */
  public boolean isCollisionDetected() {
    // Set a threshold in g's (for example, 3g).
    double thresholdG = 3.0;
    // Read accelerometer values (BuiltInAccelerometer returns values in g).
    double accelX = m_accel.getX();
    double accelY = m_accel.getY();
    double accelZ = m_accel.getZ();

    // Optionally, publish these values for debugging.
    // SmartDashboard.putNumber("Accel X (g)", accelX);
    // SmartDashboard.putNumber("Accel Y (g)", accelY);
    // SmartDashboard.putNumber("Accel Z (g)", accelZ);

    // If any axis exceeds the threshold, consider it a collision.
    return (Math.abs(accelX) > thresholdG ||
        Math.abs(accelY) > thresholdG ||
        Math.abs(accelZ) > thresholdG);
  }

  // Monitors motor current
  private final CurrentMonitor m_currentMonitor = new CurrentMonitor(m_frontLeft, m_frontRight, m_rearLeft,
      m_rearRight);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Instead of using the passed fieldRelative parameter, we use our internal
    // flag.
    boolean useFieldRelative = m_fieldOriented;

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * speedMultiplier;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * speedMultiplier;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    // Use getAdjustedHeading() for field-oriented drive.
    var chassisSpeeds = useFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeedDelivered,
            ySpeedDelivered,
            rotDelivered,
            Rotation2d.fromDegrees(getAdjustedHeading()))
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getChassisSpeed() {
    double speedFL = Math.abs(m_frontLeft.getState().speedMetersPerSecond);
    double speedFR = Math.abs(m_frontRight.getState().speedMetersPerSecond);
    double speedRL = Math.abs(m_rearLeft.getState().speedMetersPerSecond);
    double speedRR = Math.abs(m_rearRight.getState().speedMetersPerSecond);
    return (speedFL + speedFR + speedRL + speedRR) / 4.0;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  private double speedMultiplier = 1.0;

  public void setSpeedMultiplier(double multiplier) {
    this.speedMultiplier = multiplier;
    // SmartDashboard.putNumber("Drive Speed Multiplier", speedMultiplier);
  }

  private double fieldOrientationOffset = 0.0;

  public void resetFieldOrientation() {
    // Set the current heading as the new zero reference.
    fieldOrientationOffset = getHeading();

  }

  public double getMT2Heading() {
    LimelightHelpers.PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (mt2Estimate != null && mt2Estimate.tagCount > 0) {
        return mt2Estimate.pose.getRotation().getDegrees();
    }
    // Fallback to gyro heading if no valid vision estimate is available.
    return getHeading();
}


  public void initFieldOrientationForAlliance() {

    // double currentHeading = getHeading(); // in degrees
    double currentHeading = getMT2Heading();

    double allianceAdjustment = 0;
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
        allianceAdjustment = 180;
    }
    fieldOrientationOffset = Math.abs(allianceAdjustment - currentHeading);

  }


  public double getAdjustedHeading() {
    // Adjust the raw heading by subtracting the offset.
    return getHeading() - fieldOrientationOffset;
  }

  public void setDriveVoltage(double voltage) {
    // For simplicity, send the same voltage to all drive motors.
    m_frontLeft.setDrivingVoltage(voltage);
    m_frontRight.setDrivingVoltage(voltage);
    m_rearLeft.setDrivingVoltage(voltage);
    m_rearRight.setDrivingVoltage(voltage);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    if (Constants.COMP_CODE){
    }
    else {
      m_currentMonitor.update();
    SmartDashboard.putBoolean("Collision Detected", isCollisionDetected());
    SmartDashboard.putNumber("Field Orientation Offset", fieldOrientationOffset);
    SmartDashboard.putNumber("Adjusted Heading", getAdjustedHeading());
    SmartDashboard.putNumber("Heading", getHeading());

  }


}