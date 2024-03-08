// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.patterns;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LightPattern;
import frc.robot.systems.LEDLights;

/** Add your docs here. */
public class BenPattern implements LightPattern {
    private static final Color ORAGNE = new Color(1.0, .2, 0.0);
    private static final Color GREEN = new Color(0.0, .255, .0);
    private double time;

    public static double jankSineWave(double cancer) {
        return Math.abs(1.0 - cancer);
    }

    public BenPattern() {
    }

    @Override
    public Color[] getPattern(double time) {
        this.time = time;

        double jankSine = jankSineWave(time);

        Color oragneScale = new Color(ORAGNE.red * jankSine, ORAGNE.green * jankSine, ORAGNE.blue * jankSine);
        double inverse = 1.0 - jankSine;
        Color greenScale = new Color(GREEN.red * inverse, GREEN.green * inverse, GREEN.blue * inverse);
        Color colorSum = new Color(oragneScale.red + greenScale.red, oragneScale.green + greenScale.green, oragneScale.blue + greenScale.blue);
        return new Color[] { colorSum };


    }

    @Override
    public boolean getShouldResetTimer() {
        return time >= 2.0;
    }

    @Override
    public int getPatternLength() {
        return 1;
    }

    @Override
    public boolean isEqual(LightPattern pattern) {
        if (pattern == null) {
            return false;
        }
        
        if (pattern.getClass() != this.getClass()) {
            return false;
        }

        return true;
    }
    
}
