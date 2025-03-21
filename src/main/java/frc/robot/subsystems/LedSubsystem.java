// File: LedSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

public class LedSubsystem extends SubsystemBase {
    private static final int LED_PWM_PORT = 0;
    private static final int LED_COUNT = 36;

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public enum LEDMode {
        IDLE,
        ARM_HIGH,        // Top group: Back (9-12), Front (13-17)
        ARM_MID,         // Middle group: Back (5-8), Front (18-25)
        ARM_LOW,         // Bottom group: Back (0-4), Front (26-35)
        MATCH_END_FLASH,
        SOLID_GREEN,
        FLASH_GREEN
    }

    private LEDMode currentMode = LEDMode.IDLE;
    private boolean flashState = false;
    private long lastFlashTime = 0;
    private final long flashIntervalMillis = 300;

    public LedSubsystem() {
        led = new AddressableLED(LED_PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LED_COUNT);
        led.setLength(LED_COUNT);
        led.setData(ledBuffer);
        led.start();
    }

    public void setLEDMode(LEDMode mode) {
        currentMode = mode;
    }

    private Color getAllianceColor() {
        return (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red)
                ? Color.kGreen : Color.kBlue;
    }

    @Override
    public void periodic() {
        long currentTime = System.currentTimeMillis();
        Color allianceColor = getAllianceColor();

        switch (currentMode) {
            case MATCH_END_FLASH:
                if (currentTime - lastFlashTime >= flashIntervalMillis) {
                    flashState = !flashState;
                    lastFlashTime = currentTime;
                }
                for (int i = 0; i < LED_COUNT; i++) {
                    ledBuffer.setLED(i, flashState ? allianceColor : Color.kBlack);
                }
                break;

            case SOLID_GREEN:
                for (int i = 0; i < LED_COUNT; i++) {
                    ledBuffer.setLED(i, Color.kGreen);
                }
                break;

            case FLASH_GREEN:
                if (currentTime - lastFlashTime >= flashIntervalMillis) {
                    flashState = !flashState;
                    lastFlashTime = currentTime;
                }
                for (int i = 0; i < LED_COUNT; i++) {
                    ledBuffer.setLED(i, flashState ? Color.kGreen : Color.kBlack);
                }
                break;

            case ARM_HIGH:
                for (int i = 0; i < LED_COUNT; i++) {
                    ledBuffer.setLED(i, ((i >= 9 && i <= 12) || (i >= 13 && i <= 17)) ? allianceColor : Color.kBlack);
                }
                break;

            case ARM_MID:
                for (int i = 0; i < LED_COUNT; i++) {
                    ledBuffer.setLED(i, ((i >= 5 && i <= 8) || (i >= 18 && i <= 25)) ? allianceColor : Color.kBlack);
                }
                break;

            case ARM_LOW:
                for (int i = 0; i < LED_COUNT; i++) {
                    ledBuffer.setLED(i, ((i >= 0 && i <= 4) || (i >= 26 && i <= 35)) ? allianceColor : Color.kBlack);
                }
                break;

                case IDLE:
                default:
                    // Calculate a brightness value that oscillates between 0 and 1 over a 2-second period.
                    double fadePeriod = 2000.0; // period in milliseconds (2 seconds)
                    double t = (System.currentTimeMillis() % fadePeriod) / fadePeriod; // t in [0, 1)
                    // Using sine wave shifted so brightness goes from 0 to 1 smoothly:
                    double brightness = 0.5 * (1 + Math.sin(2 * Math.PI * t - Math.PI / 2));
                    
                    // Create a new color by scaling each channel of the allianceColor by the brightness factor.
                    // Note: The WPILib Color class components range from 0 to 1.
                    Color fadedColor = new Color(
                            allianceColor.red * brightness,
                            allianceColor.green * brightness,
                            allianceColor.blue * brightness
                    );
                    
                    // Set all LEDs to the faded color.
                    for (int i = 0; i < LED_COUNT; i++) {
                        ledBuffer.setLED(i, fadedColor);
                    }
                    break;
                
        }

        led.setData(ledBuffer);
        if (Constants.COMP_CODE){
        // Not really interested in seeing anything during comp for this one.
        }
        else{
        SmartDashboard.putString("LED Mode", currentMode.toString());
        }
    }
}
