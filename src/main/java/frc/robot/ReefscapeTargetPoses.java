// File: TargetPoses.java
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ReefscapeTargetPoses {
    // Blue alliance processor target: x = 11.529, y = 7.399, theta = -180° (-π radians)
    public static final Pose2d BlueProcessor = new Pose2d(11.529, 7.399, new Rotation2d(-Math.PI));
    
    // Red alliance processor target: x = 6.099, y = 0.595, theta = 0° (0 radians)
    public static final Pose2d RedProcessor = new Pose2d(6.099, 0.595, new Rotation2d(0));

    // Red Alliance Targets (Tags 6-11)
    public static final Pose2d RED_TAG6_RIGHT = new Pose2d(13.83, 3.060, new Rotation2d(Math.toRadians(240)));
    public static final Pose2d RED_TAG6_LEFT  = new Pose2d(13.566, 2.875, new Rotation2d(Math.toRadians(240)));
    
    public static final Pose2d RED_TAG7_RIGHT = new Pose2d(14.27, 4.210, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d RED_TAG7_LEFT  = new Pose2d(14.27, 3.850, new Rotation2d(Math.toRadians(180)));
    
    public static final Pose2d RED_TAG8_RIGHT = new Pose2d(13.52, 5.170, new Rotation2d(Math.toRadians(240)));
    public static final Pose2d RED_TAG8_LEFT  = new Pose2d(13.82, 4.990, new Rotation2d(Math.toRadians(240)));
    
    public static final Pose2d RED_TAG9_RIGHT = new Pose2d(12.30, 4.990, new Rotation2d(Math.toRadians(300)));
    public static final Pose2d RED_TAG9_LEFT  = new Pose2d(12.60, 5.170, new Rotation2d(Math.toRadians(300)));
    
    public static final Pose2d RED_TAG10_RIGHT = new Pose2d(11.829, 4.210, new Rotation2d(Math.toRadians(360)));
    public static final Pose2d RED_TAG10_LEFT  = new Pose2d(11.829, 3.850, new Rotation2d(Math.toRadians(360)));
    
    public static final Pose2d RED_TAG11_RIGHT = new Pose2d(12.588, 2.812, new Rotation2d(Math.toRadians(60)));
    public static final Pose2d RED_TAG11_LEFT  = new Pose2d(12.285, 2.989, new Rotation2d(Math.toRadians(60)));
    
    // Blue Alliance Targets (Tags 17-22)
    public static final Pose2d BLUE_TAG17_RIGHT = new Pose2d(4.041, 2.889, new Rotation2d(Math.toRadians(-300)));
    public static final Pose2d BLUE_TAG17_LEFT  = new Pose2d(3.731, 3.065, new Rotation2d(Math.toRadians(-300)));
    
    public static final Pose2d BLUE_TAG18_RIGHT = new Pose2d(4.041, 4.205, new Rotation2d(Math.toRadians(-300)));
    public static final Pose2d BLUE_TAG18_LEFT  = new Pose2d(4.041, 3.848, new Rotation2d(Math.toRadians(-300)));
    
    public static final Pose2d BLUE_TAG19_RIGHT = new Pose2d(3.731, 4.990, new Rotation2d(Math.toRadians(-300)));
    public static final Pose2d BLUE_TAG19_LEFT  = new Pose2d(4.041, 5.170, new Rotation2d(Math.toRadians(-300)));
    
    public static final Pose2d BLUE_TAG20_RIGHT = new Pose2d(4.942, 5.170, new Rotation2d(Math.toRadians(-300)));
    public static final Pose2d BLUE_TAG20_LEFT  = new Pose2d(5.253, 4.511, new Rotation2d(Math.toRadians(-300)));
    
    public static final Pose2d BLUE_TAG21_RIGHT = new Pose2d(5.705, 4.205, new Rotation2d(Math.toRadians(-300)));
    public static final Pose2d BLUE_TAG21_LEFT  = new Pose2d(5.705, 3.848, new Rotation2d(Math.toRadians(-300)));
    
    public static final Pose2d BLUE_TAG22_RIGHT = new Pose2d(5.253, 3.065, new Rotation2d(Math.toRadians(-300)));
    public static final Pose2d BLUE_TAG22_LEFT  = new Pose2d(4.942, 2.889, new Rotation2d(Math.toRadians(-300)));

    
    // Add additional target poses as needed.
}
