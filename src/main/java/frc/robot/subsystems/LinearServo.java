package frc.robot.subsystems;

import frc.robot.SmartPrintable;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LinearServo extends SmartPrintable {
    private final Servo servo;

    /**
     * Creates a new `LinearServo` given a PWM channel;
     */
    public LinearServo(int servoChannel) {
        servo = new Servo(servoChannel);
    }

    /**
     * Gets the set position of the servo, not its actual position, the servo
     * does not report position.
     */
    public double position() {
        return servo.get();
    }

    /**
     * Sets the servo's position.
     */
    public void setPosition(double position) {
        position = position > 1.0
            ? 1.0
            : position;

        position = position < 0.0
            ? 0.0
            : position;

        servo.set(position);
    }

    @Override
    public void print() {
        SmartDashboard.putNumber("Linear Servo (on channel " + servo.getChannel() + ") position", servo.get());
    }
}

