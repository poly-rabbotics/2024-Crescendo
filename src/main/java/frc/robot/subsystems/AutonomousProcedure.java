package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Function;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SmartPrintable;

public class AutonomousProcedure extends SmartPrintable implements Runnable {
    private ArrayList<ProcedureStep> procedureSteps = new ArrayList<ProcedureStep>();
    private int procedureStep = 0;
    private String name = null;

    public AutonomousProcedure(String name) {
        this.name = name;
    }

    /**
     * Adds a step that will only run upon the previous's completion. The passed
     * is `CompletionState` may safely be ignored and should always represent a
     * completed step.
     */
    public AutonomousProcedure wait(Function<StepStatus, StepStatus> fn) {
        procedureSteps.add(new ProcedureStep(fn, true));
        return this;
    }

    /**
     * Adds a step that will only run regardless of the previous's completion.
     * It is expected that the given function will be able to manage itself
     * based on the given `CompletionState` of the previous step.
     */
    public AutonomousProcedure then(Function<StepStatus, StepStatus> fn) {
        procedureSteps.add(new ProcedureStep(fn, false));
        return this;
    }

    @Override
    public void run() {
        StepStatus prevState = StepStatus.Done;

        for (ProcedureStep step : procedureSteps) {
            prevState = step.run(prevState);
        }
    }

    @Override
    public void print() {
        SmartDashboard.putNumber("Autonomous Procedure (" + name + ") step number", procedureStep);
    }

    /**
     * Step of a procedure, handling the state in which it starts.
     */
    public class ProcedureStep {
        private Function<StepStatus, StepStatus> fn;
        private StepStatus prevState = StepStatus.Waiting;
        private boolean isWaitingStep;

        public ProcedureStep(Function<StepStatus, StepStatus> fn, boolean isWaitingStep) {
            this.fn = fn;
            this.isWaitingStep = isWaitingStep;
        }

        public StepStatus run(StepStatus previousStep) {
            if (prevState == StepStatus.Done) {
                return prevState;
            }
            
            if (isWaitingStep && previousStep != StepStatus.Done) {
                return StepStatus.Waiting;
            }

            return fn.apply(previousStep);
        }
    }

    public enum StepStatus {
        Waiting,
        Running,
        Done
    }
}
