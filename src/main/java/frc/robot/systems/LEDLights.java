package frc.robot.systems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.patterns.Rainbow;
import frc.robot.patterns.Solid;
import frc.robot.patterns.Breathe;
import frc.robot.patterns.FadeIn;
import frc.robot.patterns.FadeIntoPattern;
import frc.robot.subsystems.ColorUtils;
import frc.robot.subsystems.LightPattern;
import frc.robot.subsystems.LightRenderer;
import frc.robot.subsystems.SwerveMode;

public class LEDLights {
    // Public since it may be usefule for pattern instantiation.
    public static final int LED_LENGTH = 64;
    private static final int LED_PORT = 8;

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
            setPattern = new Rainbow(100, 1.0);
            //setPattern = new Breathe();
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

        setPattern = new Rainbow(100, 100);

        if (setPattern != null) {
            instance.renderer.setIfNotEqual(setPattern);
        }

        instance.renderer.run();
    }

    /**
	 * Sets bit arrangements of colors in patterns by an array with equal 
	 * indices to the LED each bit arrangement wishes to address. If given an
	 * array that does not fill the whole strip, RGB will be assumed for all 
	 * remaining lights.
	 */
    public static void setBitArrangements(ColorUtils.BitArrangement[] arrangements) {
        instance.renderer.setBitArrangements(arrangements);
    }
}