package frc.robot.systems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.patterns.Rainbow;
import frc.robot.patterns.FadeIn;
import frc.robot.patterns.FadeIntoPattern;
import frc.robot.subsystems.LightPattern;
import frc.robot.subsystems.LightRenderer;
import frc.robot.subsystems.SwerveMode;

public class LEDLights {
    // Public since it may be usefule for pattern instantiation.
    public static final int LED_LENGTH = 108;
    private static final int LED_PORT = 1;

    private static final LEDLights instance = new LEDLights();

    private final LightRenderer renderer;
    //private final ScheduledExecutorService executorService;

    private LEDLights() {
        renderer = new LightRenderer(LED_PORT, LED_LENGTH);
        renderer.setPattern(new Rainbow(69, 100));
    }

    public static void run() {
        LightPattern setPattern = null;

        if (DriverStation.isDisabled()) {
            // Rainbow if disabled.
            //setPattern = new Rainbow();
            setPattern = new FadeIntoPattern(new Rainbow(), 0.15);
        } else if (DriverStation.isAutonomous()) {
            setPattern = new FadeIn(new Color(0.0, 1.0, 0.0), 1.0);
        } else {
            if (SwerveDrive.getDisplayMode() == SwerveMode.ROCK) {
                // If in rock mode make wyvern scary >:D
                setPattern = new FadeIn(new Color(1.0, 0.0, 0.0), 1.0);
            } else if (SwerveDrive.getDisplayMode() == SwerveMode.AIMBOT) {
                setPattern = new FadeIn(new Color(0.0, 1.0, 0.0), 1.0);
            } else if (SwerveDrive.getDisplayMode() == SwerveMode.AIMBOT_ROTATION) {
                setPattern = new FadeIn(new Color(0.0, 0.4, 1.0), 1.0);
            }
        }

        if (setPattern != null) {
            instance.renderer.setIfNotEqual(setPattern);
        }

        instance.renderer.run();
    }
}