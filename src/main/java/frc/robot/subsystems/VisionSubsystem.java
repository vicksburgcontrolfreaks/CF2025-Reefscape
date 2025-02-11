// File: VisionSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;  // Assume this contains TEAM_NUMBER

public class VisionSubsystem extends SubsystemBase {
    // Limelight network table reference
    private final NetworkTable limelightTable;
    
    // USB camera for the rear webcam stream
    private final UsbCamera rearWebCam;

    public VisionSubsystem() {
        // Get the Limelight's network table (assumed to be "limelight")
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        // Start automatic capture for a USB camera (device number 1).
        rearWebCam = CameraServer.startAutomaticCapture("rearWebCam", 1);
        rearWebCam.setResolution(640, 480);

        // Obtain the camera's registered name.
        String cameraName = rearWebCam.getName();

        // Construct the camera URL using your team number from Constants.
        String cameraUrl = "http://roborio-" + Constants.TEAM_NUMBER + "-FRC.local:1181/?action=stream";

        // Add the camera stream to the Shuffleboard "Vision" tab.
        Shuffleboard.getTab("Vision").addCamera("Rear Webcam", cameraName, cameraUrl);
    }

    @Override
    public void periodic() {
        // Retrieve standard Limelight values.
        double tv = limelightTable.getEntry("tv").getDouble(0.0);
        double tx = limelightTable.getEntry("tx").getDouble(0.0);
        double ty = limelightTable.getEntry("ty").getDouble(0.0);
        double ta = limelightTable.getEntry("ta").getDouble(0.0);

        if (tv == 1.0) {  // Target detected
            SmartDashboard.putNumber("Limelight TX", tx);
            SmartDashboard.putNumber("Limelight TY", ty);
            SmartDashboard.putNumber("Limelight Target Area", ta);
        } else {
            SmartDashboard.putString("Limelight Status", "No Target Detected");
        }
    }
    
    // Accessor methods for auto-align commands:
    public double getTx() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }
    
    public double getTa() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }
}
