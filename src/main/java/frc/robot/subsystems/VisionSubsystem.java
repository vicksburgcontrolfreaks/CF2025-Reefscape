// File: VisionSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    public static final boolean USE_LIMELIGHT = true;

    /*
     * offsets for high mount position:
     * LL Forward: -0.212
     * LL Right: 0.174
     * LL Up: 0.701
     * 
     * offsets for low mount:
     * LL Forward: 0.25
     * LL Right: 0.137
     * LL Up: 0.238
     */

    public VisionSubsystem() {
        // Get the Limelight's network table (assumed to be "limelight")

        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    }

    @Override
    public void periodic() {

        // Optionally, continuously enforce the desired IMU mode.
        LimelightHelpers.SetIMUMode("limelight", 0);

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
            // SmartDashboard.putNumber("Limelight TX", tx);
            // SmartDashboard.putNumber("Limelight TY", ty);
            // SmartDashboard.putNumber("Limelight Target Area", ta);
            SmartDashboard.putNumber("Detected Tag ID", getDetectedTagIDFromNT());
        } else {
            SmartDashboard.putNumber("Detected Tag ID", 0);
        }
    }

    // Accessor methods for auto-align or command usage:
    public double getTx() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public double getTa() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    public int getDetectedTagIDFromNT() {
        double idDouble = limelightTable.getEntry("tid").getDouble(0);
        return (int) idDouble;
    }
}
