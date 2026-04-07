package frc.robot.subsystems;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    
    private Runnable m_currentPattern;
    private int m_rainbowFirstPixelHue = 0;

    public Leds(int port, int length) {
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(length);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();

        // Mode par défaut
        m_currentPattern = () -> solid(Color.kGreen);
    }

    @Override
    public void periodic() {
        if (m_currentPattern != null) {
            m_currentPattern.run();
        }
        m_led.setData(m_ledBuffer);
    }

    // --- MÉTHODES DE BASES (INTERNES) ---

    private void solid(Color color) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, color);
        }
    }

    private void strobe(Color color, double duration) {
        boolean on = (Timer.getFPGATimestamp() % duration) > (duration / 2.0);
        solid(on ? color : Color.kBlack);
    }

    public void setStrobe(Color color, double speed) {
        m_currentPattern = () -> strobe(color, speed);
    }

    public void setStrobeAlliance(double speed) {
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        Color color = Color.kGreen;
        if (alliance.isPresent()) {
            color = (alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) 
                    ? Color.kRed : Color.kBlue;
        }
        setStrobe(color, speed);
    }

    private void breath(Color color) {
        double sin = Math.sin(Timer.getFPGATimestamp() * 3.0); // Vitesse de respiration
        double intensity = (sin + 1.0) / 2.0; // Normalise entre 0 et 1
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, (int)(color.red * 255 * intensity), 
                                  (int)(color.green * 255 * intensity), 
                                  (int)(color.blue * 255 * intensity));
        }
    }

    // --- COMMANDES PUBLIQUES ---

    /** Couleur fixe via l'objet WPILib Color (ex: Color.kRed) */
    public Command setSolidColorCommand(Color color) {
        return runOnce(() -> m_currentPattern = () -> solid(color));
    }

    /** Arc-en-ciel qui défile */
    public Command setRainbowCommand() {
        return runOnce(() -> m_currentPattern = () -> {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
                m_ledBuffer.setHSV(i, hue, 255, 128);
            }
            m_rainbowFirstPixelHue = (m_rainbowFirstPixelHue + 3) % 180;
        });
    }

    /** Clignotement rapide */
    public Command setStrobeCommand(Color color) {
        return runOnce(() -> m_currentPattern = () -> strobe(color, 0.2));
    }

    /** Effet de "respiration" (fondu) */
    public Command setBreathCommand(Color color) {
        return runOnce(() -> m_currentPattern = () -> breath(color));
    }

    /** Mode dynamique selon une condition (ex: Alliance) */
    public Command setDynamicAllianceCommand(Supplier<Color> colorSupplier) {
        return runOnce(() -> m_currentPattern = () -> solid(colorSupplier.get()));
    }

    public Command setOffCommand() {
        return runOnce(() -> m_currentPattern = () -> solid(Color.kBlack));
    }
}