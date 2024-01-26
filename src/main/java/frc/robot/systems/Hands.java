/*
 * Introducing HANDS:
 * Helpful
 * Actuating
 * Note
 * Delivery
 * System
 * !!!!!!!!!!
 */

package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LinearActuator;
import frc.robot.subsystems.LinearServo;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Loader;


public class Hands extends SmartPrinter {
    private static Hands instance = new Hands();

    private static final int LINEAR_ACTUATOR_ID = 61;
    private static final int LINEAR_SERVO_ID = 9;

    private static final int SHOOTER_LEFT_MOTOR_ID = 5;
    private static final int SHOOTER_RIGHT_MOTOR_ID = 6;

    private static final int LOADER_ID = 60;

    private static LinearActuator linearActuator;
    private static LinearServo linearServo;
    private static Shooter shooter;
    private static Loader loader;
    
    private Hands() {
        super();

        linearActuator = new LinearActuator(LINEAR_ACTUATOR_ID, 108, 0.1, 0.0, 0.0);
        //linearServo = new LinearServo(LINEAR_SERVO_ID);
        loader = new Loader(LOADER_ID);

        shooter = new Shooter(SHOOTER_LEFT_MOTOR_ID, SHOOTER_RIGHT_MOTOR_ID);

    }

    public static void run(boolean intake, boolean outtake, boolean shoot, boolean runLoader, double actuatorPos) {
        shooter.run(shoot);
        loader.run(runLoader);

        linearActuator.setPosition(actuatorPos);
        linearActuator.run();
        SmartDashboard.putNumber("Loader position", loader.getEncoderPosition());

    }

    public static void print() {
        SmartDashboard.putNumber("Loader Motor Position", loader.getEncoderPosition());
    }
}
