package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.SmartPrintable;



public class Intake extends SmartPrintable {

    public enum IntakeState {
        IDLE, INTAKING, OUTTAKING
    }
    
    private static final Intake instance = new Intake();

    private static IntakeState currentState = IntakeState.IDLE;

    private static TalonFX intakeMotor;

    private static double intakeSpeed = 0.9; //intakeSpeed in RPM
    private static boolean stopped = true;

    
    private Intake() {
        intakeMotor = new TalonFX(7);
    }

    public static void run(boolean intake, boolean outtake) {
        if(intake) {
            currentState = IntakeState.INTAKING;
        } else if(outtake) {
            currentState = IntakeState.OUTTAKING;
        } else {
            currentState = IntakeState.IDLE;
        }

        if(currentState.equals(IntakeState.INTAKING)) {
            intakeMotor.set(intakeSpeed);
        } else if(currentState.equals(IntakeState.OUTTAKING)) {
            intakeMotor.set(-intakeSpeed);
        } else {
            intakeMotor.set(0);
        }
    }

    /**
     * More rudamentary method that allows for adjustment of the intake speed
     * @param runIntake Hold the button to run the intake
     */
    public static void runTest(boolean up, boolean down, boolean resume, boolean stop) {

        if(up) {
            intakeSpeed += 0.1;
        } else if(down) {
            intakeSpeed -= 0.1;
        }

        if(resume) {
            stopped = false;
        } else if(stop) {
            stopped = true;
        }

        
        if(!stopped) intakeMotor.set(intakeSpeed);
        else intakeMotor.set(0);

    }

    /**
     * Returns the current state of the intake
     * @return The current state of the intake
     */
    public static IntakeState getState() {
        return currentState;
    }

    public void print() {
        SmartDashboard.putNumber("Target Speed", intakeSpeed);
        SmartDashboard.putNumber("Motor Speed", intakeMotor.get());
    }
}
