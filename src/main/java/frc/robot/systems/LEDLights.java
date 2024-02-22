package frc.robot.systems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.patterns.Rainbow;
import frc.robot.patterns.Solid;
import frc.robot.patterns.FadeIn;
import frc.robot.patterns.FadeIntoPattern;
import frc.robot.subsystems.LightPattern;
import frc.robot.subsystems.LightRenderer;
import frc.robot.subsystems.SwerveMode;

public class LEDLights {
    // Public since it may be usefule for pattern instantiation.
    public static final int LED_LENGTH = 16;
    private static final int LED_PORT = 9; // TODO: correct

    private static final LEDLights instance = new LEDLights();

    private final LightRenderer renderer;
    //private final ScheduledExecutorService executorService;

    private LEDLights() {
        renderer = new LightRenderer(LED_PORT, LED_LENGTH, false);
        renderer.setPattern(new Solid(new Color(0.0, 0, 1.0)));
    }

    public static void run() {
        instance.renderer.run();
    }
}