package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Function;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SmartPrintable;

public class AutonomousProcedure extends SmartPrintable implements Runnable {
    private ArrayList<ProcedureStep> procedureSteps;
    private int procedureStep = 0;
    private String name = null;

    public AutonomousProcedure(String name) {
        procedureSteps = new ArrayList<ProcedureStep>();
        this.name = name;
    }

    /**
     * Adds a step that will only run upon the previous's completion. The passed
     * is `CompletionState` may safely be ignored and should always represent a
     * completed step.
     */
    public AutonomousProcedure wait(Function<CompletionState, CompletionState> fn) {
        procedureSteps.add(new ProcedureStep(fn, true));
        return this;
    }

    /**
     * Adds a step that will only run regardless of the previous's completion.
     * It is expected that the given function will be able to manage itself
     * based on the given `CompletionState` of the previous step.
     */
    public AutonomousProcedure then(Function<CompletionState, CompletionState> fn) {
        procedureSteps.add(new ProcedureStep(fn, false));
        return this;
    }

    @Override
    public void run() {
        CompletionState prevState = new CompletionState(StepStatus.Done, 1.0);

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
        private Function<CompletionState, CompletionState> fn;
        private CompletionState prevState = new CompletionState(StepStatus.Waiting, 0.0);
        private boolean isWaitingStep;

        public ProcedureStep(Function<CompletionState, CompletionState> fn, boolean isWaitingStep) {
            this.fn = fn;
            this.isWaitingStep = isWaitingStep;
        }

        public CompletionState run(CompletionState previousStep) {
            if (prevState.status == StepStatus.Done) {
                return prevState;
            }
            
            if (isWaitingStep && previousStep.status != StepStatus.Done) {
                return new CompletionState(StepStatus.Waiting, 0.0);
            }

            return fn.apply(previousStep);
        }
    }

    /**
     * Represents the state of a procedure step. If the status is
     * `StepStatus.Done` then `percentComplete` will not be considered.
     */
    public class CompletionState {
        public StepStatus status;
        public double percentComplete;

        public CompletionState(StepStatus status, double percentComplete) {
            this.status = status;
            this.percentComplete = percentComplete;
        }
    }
    
    public enum StepStatus {
        Waiting,
        Running,
        Done
    }
}
