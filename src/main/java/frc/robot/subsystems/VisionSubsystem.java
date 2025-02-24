// File: VisionSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers; // Ensure you have the appropriate helper class
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    public static final boolean USE_LIMELIGHT = true;

    public VisionSubsystem() {
        // Get the Limelight's network table (assumed to be "limelight")

        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    }

    @Override
    public void periodic() {

        // Optionally, continuously enforce the desired IMU mode.
        LimelightHelpers.SetIMUMode("limelight", 4);

        // Check if the Limelight is publishing values.
        if (limelightTable.getEntry("tv").getValue() == null) {
            SmartDashboard.putString("Limelight Status", "Limelight Not Connected");
            return;
        }

        double tv = limelightTable.getEntry("tv").getDouble(0.0);
        double tx = limelightTable.getEntry("tx").getDouble(0.0);
        double ty = limelightTable.getEntry("ty").getDouble(0.0);
        double ta = limelightTable.getEntry("ta").getDouble(0.0);

        if (tv == 1.0) { // Target detected
            SmartDashboard.putNumber("Limelight TX", tx);
            SmartDashboard.putNumber("Limelight TY", ty);
            SmartDashboard.putNumber("Limelight Target Area", ta);
        } else {
            SmartDashboard.putString("Limelight Status", "No Target Detected");
        }

    }

    // Accessor methods for auto-align or command usage:
    public double getTx() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public double getTa() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    /**
     * Returns a placeholder AprilTag relative pose if a target is detected.
     * In a complete implementation, you would use camera calibration data to
     * compute this.
     */
    public Pose2d getAprilTagPoseRelative() {
        double tv = limelightTable.getEntry("tv").getDouble(0.0);
        if (tv < 1.0) {
            return null;
        }
        // Placeholder: return a pose 1 meter ahead of the robot.
        return new Pose2d(1.0, 0.0, new Rotation2d(0));
    }

    public int getDetectedTagIDFromNT() {
        double idDouble = limelightTable.getEntry("tid").getDouble(0);
        return (int) idDouble;
    }
}
