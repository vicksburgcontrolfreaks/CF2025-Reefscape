// Control Freaks 2025

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class CoralArmSubsystem extends SubsystemBase {

    private final SparkMax m_armExtend;

    public CoralArmSubsystem() {
        m_armExtend = new SparkMax(ArmConstants.kCoralExtendCanId, MotorType.kBrushless);

    }

    public void moveArm(double speed) {
        m_armExtend.set(speed);
    }

    public void stopArm() {
        m_armExtend.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // You can use it for telemetry, diagnostics, etc.
    }
}
