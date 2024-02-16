package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class KrakenTest {
    private static TalonFX kraken1 = new TalonFX(0);
    private static TalonFX kraken2 = new TalonFX(1);

    public static void runKraken1(double speed) {
        kraken1.set(speed);
    }

    public static void runKraken2(double speed) {
        kraken2.set(speed);
    }
}
