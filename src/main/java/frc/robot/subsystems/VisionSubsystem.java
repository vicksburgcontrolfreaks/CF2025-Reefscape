// File: VisionSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
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
        // Get the Limelight's network table
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
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

    @Override
    public void periodic() {

        double tv = limelightTable.getEntry("tv").getDouble(0.0);
        double tx = limelightTable.getEntry("tx").getDouble(0.0);
        double ty = limelightTable.getEntry("ty").getDouble(0.0);
        double ta = limelightTable.getEntry("ta").getDouble(0.0);

        if (Constants.COMP_CODE) {
            if (tv == 1.0) { // Target detected
                SmartDashboard.putNumber("Detected Tag ID", getDetectedTagIDFromNT());
            } else {
                SmartDashboard.putNumber("Detected Tag ID", 0);
            }
        } else {
            //additional debugging calls here
            if (tv == 1.0) { // Target detected
                SmartDashboard.putNumber("Detected Tag ID", getDetectedTagIDFromNT());
            } else {
                SmartDashboard.putNumber("Detected Tag ID", 0);
            }
        }
    }
}
