// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double TEAM_NUMBER = 8126;

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21.495);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21.495);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs

    public static final int kFrontLeftDrivingCanId = 8; // 8
    public static final int kFrontLeftTurningCanId = 9; // 9

    public static final int kFrontRightDrivingCanId = 2; // 2
    public static final int kFrontRightTurningCanId = 3; // 3

    public static final int kRearLeftDrivingCanId = 6; // 6
    public static final int kRearLeftTurningCanId = 7; // 7

    public static final int kRearRightDrivingCanId = 4; // 4
    public static final int kRearRightTurningCanId = 5; // 5

    public static final boolean kGyroReversed = false;
  }

  public static final class ArmConstants {
    public static final int kCoralExtendCanId = 10;
    public static final int kCoralAngleExtenderCanId = 11;
    public static final int AlgaeExtenderCanId = 12;
    public static final int AlgaeCollectorCanId = 13;
    public static final int CoralCollectorCanId = 14;
    public static final int HarpoonCanId = 15;

    public static final int TGT_INIT = 0; // enumeration do not change
    public static final int TGT_LOW = 1; // enumeration do not change
    public static final int TGT_MID = 2; // enumeration do not change
    public static final int TGT_HIGH = 3; // enumeration do not change

    // Coral Extender
    public static final double lowTgtHeight  =  -35; //target offset from init -40.28
    public static final double midTgtHeight  =  -58.5; //target offset from init -72.31
    public static final double highTgtHeight = -100;//target offset from init -103.72

    public static final double CE_PGain = 0.01; // 0.02
    public static final double CE_IGain = 0.01;
    public static final double CE_I_MAX = 8.0;
    public static final double CE_MAX = 0.55;

    // Coral Arm Angle  
    public static final double lowTgtAngle  =  0.42; //0.44 target offset from init 39.28
    public static final double midTgtAngle  =  0.21; //target offset from init 21.93
    public static final double highTgtAngle =  0.127; //target offset from init 10.93

    public static final double CA_PGain = 2.0;  //0.01
    public static final double CA_IGain = 0.03;  //0.002
    public static final double CA_I_MAX = 0.35;
    public static final double CA_MAX   = 0.16;  //0.25 was looking ok
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    // public static final double kDrivingMotorFreeSpeedRps =
    // NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kDrivingMotorFreeSpeedRps = NeoVortexMotorConstants.kFreeSpeedRpm / 60;

    public static final double kWheelDiameterMeters = 0.0762; // 3in
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kMechanismControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 6.0; // 4.0
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.0; // 1.0
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kPXController = 1.2; // 1.0
    public static final double kPYController = 1.2; // 1.0
    public static final double kPThetaController = 1.2; // 1.0

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // PID gains for forward (distance) and lateral control (tuning these is
    // necessary).
    public static final double kPForward = 1.0; // Example value; adjust by testing
    public static final double kPLateral = 1.5; // Example value; adjust by testing

    // APRIL TAG: Tolerances to determine when alignment is good enough.

    public static final double kForwardTolerance = Units.inchesToMeters(2); // ~0.1524 m
    public static final double kLateralTolerance = 1.0; // Acceptable error in tx (in degrees)
  }

  public static final class NeoVortexMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class AlgaeConstants {
    public static double extended = 70;
    public static double ready = 20;
    public static double stowed = 0;
  }

  public static final class ReefscapeTargetPoses {
    // Red Alliance Targets
    public static final Pose2d RED_PROCESSOR = new Pose2d(6.099, 0.595, new Rotation2d(0));
    public static final Pose2d RED_CORAL_STATION_0 = new Pose2d(16.000, 0.670, new Rotation2d(-235));
    public static final Pose2d RED_CORAL_STATION_1 = new Pose2d(16.000, 7.370, new Rotation2d(0));

    public static final Pose2d RED_TAG6_RIGHT = new Pose2d(13.83, 3.060, new Rotation2d(Math.toRadians(120)));
    public static final Pose2d RED_TAG6_LEFT = new Pose2d(13.566, 2.875, new Rotation2d(Math.toRadians(120)));

    public static final Pose2d RED_TAG7_RIGHT = new Pose2d(14.27, 4.210, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d RED_TAG7_LEFT = new Pose2d(14.27, 3.850, new Rotation2d(Math.toRadians(180)));

    public static final Pose2d RED_TAG8_RIGHT = new Pose2d(13.52, 5.170, new Rotation2d(Math.toRadians(240)));
    public static final Pose2d RED_TAG8_LEFT = new Pose2d(13.82, 4.990, new Rotation2d(Math.toRadians(240)));

    public static final Pose2d RED_TAG9_RIGHT = new Pose2d(12.30, 4.990, new Rotation2d(Math.toRadians(300))); 
    public static final Pose2d RED_TAG9_LEFT = new Pose2d(12.60, 5.170, new Rotation2d(Math.toRadians(300))); 

    public static final Pose2d RED_TAG10_RIGHT = new Pose2d(11.81, 3.850, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d RED_TAG10_LEFT = new Pose2d(11.81, 4.210, new Rotation2d(Math.toRadians(0)));

    public static final Pose2d RED_TAG11_RIGHT = new Pose2d(12.588, 2.812, new Rotation2d(Math.toRadians(60)));
    public static final Pose2d RED_TAG11_LEFT = new Pose2d(12.285, 2.989, new Rotation2d(Math.toRadians(60)));


    // Intermediate positions that should have no obstructions to turning
    // red 0 starts by going to tag 11
    // red 1 starts by going to tag 9
    public static final Pose2d RED_0_INTA = new Pose2d(11.8, 2.2, new Rotation2d(Math.toRadians(100)));
    public static final Pose2d RED_0_INTB = new Pose2d(13.5, 1.75, new Rotation2d(Math.toRadians(100)));
    public static final Pose2d RED_1_INT = new Pose2d(12.0, 5.80, new Rotation2d(Math.toRadians(300)));

    // Blue Alliance Targets
    public static final Pose2d BLUE_PROCESSOR = new Pose2d(11.529, 7.399, new Rotation2d(-Math.PI));
    public static final Pose2d BLUE_CORAL_STATION_0 = new Pose2d(1.600, 0.660, new Rotation2d(54));
    public static final Pose2d BLUE_CORAL_STATION_1 = new Pose2d(1.600, 7.370, new Rotation2d(-54));

    public static final Pose2d BLUE_TAG17_RIGHT = new Pose2d(4.041, 2.889, new Rotation2d(Math.toRadians(60)));
    public static final Pose2d BLUE_TAG17_LEFT = new Pose2d(3.731, 3.065, new Rotation2d(Math.toRadians(60)));

    public static final Pose2d BLUE_TAG18_RIGHT = new Pose2d(4.041, 4.205, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d BLUE_TAG18_LEFT = new Pose2d(4.041, 3.848, new Rotation2d(Math.toRadians(0)));

    public static final Pose2d BLUE_TAG19_RIGHT = new Pose2d(3.731, 4.990, new Rotation2d(Math.toRadians(300))); 
    public static final Pose2d BLUE_TAG19_LEFT = new Pose2d(4.041, 5.170, new Rotation2d(Math.toRadians(300))); 

    public static final Pose2d BLUE_TAG20_RIGHT = new Pose2d(4.942, 5.170, new Rotation2d(Math.toRadians(240)));
    public static final Pose2d BLUE_TAG20_LEFT = new Pose2d(5.253, 4.511, new Rotation2d(Math.toRadians(240))); 

    public static final Pose2d BLUE_TAG21_RIGHT = new Pose2d(5.705, 4.205, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BLUE_TAG21_LEFT = new Pose2d(5.705, 3.848, new Rotation2d(Math.toRadians(180)));

    public static final Pose2d BLUE_TAG22_RIGHT = new Pose2d(5.253, 3.065, new Rotation2d(Math.toRadians(120)));
    public static final Pose2d BLUE_TAG22_LEFT = new Pose2d(4.942, 2.889, new Rotation2d(Math.toRadians(120)));

    // Intermediate positions that should have no obstructions to turning
    public static final Pose2d BLUE_0_INT = new Pose2d(5.347, 2.407, new Rotation2d(Math.toRadians(120)));
    public static final Pose2d BLUE_1_INT = new Pose2d(5.626, 5.86, new Rotation2d(Math.toRadians(240)));
    
  }
}
