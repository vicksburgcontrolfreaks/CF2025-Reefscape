// File: LedSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

public class LedSubsystem extends SubsystemBase {
    // The PWM port on the RoboRIO the LED strip is connected to.
    private static final int LED_PWM_PORT = 9;
    // Total number of LEDs on the strip.
    private static final int LED_COUNT = 36;

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public enum LEDMode {
        IDLE,
        ARM_HIGH,        // Top group in each half (left indices 12–17, right indices 18–23)
        ARM_MID,         // Middle group in each half (left indices 6–11, right indices 24–29)
        ARM_LOW,         // Bottom group in each half (left indices 0–5, right indices 30–35)
        MATCH_END_FLASH, // Entire strip flashes red.
        SOLID_GREEN,     // Entire strip solid green.
        FLASH_GREEN      // Entire strip flashing green.
    }

    private LEDMode currentMode = LEDMode.IDLE;
    private boolean flashState = false;
    private long lastFlashTime = 0;
    private final long flashIntervalMillis = 300; // Flash every 300ms.

    public LedSubsystem() {
        led = new AddressableLED(LED_PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LED_COUNT);
        led.setLength(LED_COUNT);
        led.setData(ledBuffer);
        led.start();
    }

    /**
     * Sets the current LED mode.
     * @param mode The desired LED mode.
     */
    public void setLEDMode(LEDMode mode) {
        currentMode = mode;
    }

    /**
     * Returns the color based on alliance assignment.
     * Red alliance → red, Blue alliance → blue.
     */
    private Color getAllianceColor() {
        return (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red)
                ? Color.kRed : Color.kBlue;
    }

    @Override
    public void periodic() {
        long currentTime = System.currentTimeMillis();
        Color allianceColor = getAllianceColor();

        switch (currentMode) {
            case MATCH_END_FLASH:
                // Entire strip flashes red.
                if (currentTime - lastFlashTime >= flashIntervalMillis) {
                    flashState = !flashState;
                    lastFlashTime = currentTime;
                }
                for (int i = 0; i < LED_COUNT; i++) {
                    ledBuffer.setLED(i, flashState ? allianceColor : Color.kBlack);
                }
                break;

            case SOLID_GREEN:
                // Entire strip solid green.
                for (int i = 0; i < LED_COUNT; i++) {
                    ledBuffer.setLED(i, Color.kGreen);
                }
                break;

            case FLASH_GREEN:
                // Entire strip flashing green.
                if (currentTime - lastFlashTime >= flashIntervalMillis) {
                    flashState = !flashState;
                    lastFlashTime = currentTime;
                }
                for (int i = 0; i < LED_COUNT; i++) {
                    ledBuffer.setLED(i, flashState ? Color.kGreen : Color.kBlack);
                }
                break;

            case ARM_HIGH:
                // Left half: top group (indices 12–17) = allianceColor; rest off.
                for (int i = 0; i < 18; i++) {
                    ledBuffer.setLED(i, (i >= 12 && i < 18) ? allianceColor : Color.kBlack);
                }
                // Right half: top group (indices 18–23) = allianceColor; rest off.
                for (int i = 18; i < LED_COUNT; i++) {
                    ledBuffer.setLED(i, (i >= 18 && i < 24) ? allianceColor : Color.kBlack);
                }
                break;

            case ARM_MID:
                // Left half: middle group (indices 6–11) = allianceColor.
                for (int i = 0; i < 18; i++) {
                    ledBuffer.setLED(i, (i >= 6 && i < 12) ? allianceColor : Color.kBlack);
                }
                // Right half: middle group (indices 24–29) = allianceColor.
                for (int i = 18; i < LED_COUNT; i++) {
                    ledBuffer.setLED(i, (i >= 24 && i < 30) ? allianceColor : Color.kBlack);
                }
                break;

            case ARM_LOW:
                // Left half: bottom group (indices 0–5) = allianceColor.
                for (int i = 0; i < 18; i++) {
                    ledBuffer.setLED(i, (i < 6) ? allianceColor : Color.kBlack);
                }
                // Right half: bottom group (indices 30–35) = allianceColor.
                for (int i = 18; i < LED_COUNT; i++) {
                    ledBuffer.setLED(i, (i >= 30 && i < 36) ? allianceColor : Color.kBlack);
                }
                break;

            case IDLE:
            default:
                // Turn off all LEDs.
                for (int i = 0; i < LED_COUNT; i++) {
                    ledBuffer.setLED(i, Color.kBlack);
                }
                break;
        }

        led.setData(ledBuffer);
        SmartDashboard.putString("LED Mode", currentMode.toString());
    }
}
