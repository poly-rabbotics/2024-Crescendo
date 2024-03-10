package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Function;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SmartPrintable;

public class AutonomousProcedure extends SmartPrintable implements Runnable {
    private ArrayList<ProcedureStep> procedureSteps = new ArrayList<ProcedureStep>();
    private int procedureStep = 0;
    private String name = null;

    // This exists only for nested procedures and is used to inject the
    // completion of the prior step from before the nested procedure, ensuring
    // that "wait" steps work as intended and that "then" steps get accurate
    // data.
    private StepStatus startingStepStatus = StepStatus.Done;
    private StepStatus lastStepStatus = StepStatus.Waiting;

    public AutonomousProcedure(String name) {
        this.name = name;
    }

    public void reset() {
        lastStepStatus = StepStatus.Waiting;
        startingStepStatus = StepStatus.Done;
        procedureStep = 0;
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
     * Adds all steps of another procedure to this one. This function ensures
     * that the enclosed procedure will operate the same as though its steps
     * were individually added to this procedure with the exception that the
     * previous step must complete before the procedure, regardless of whether
     * or not the first step in the nested procedure is a "then" step or not.
     */
    public AutonomousProcedure wait(AutonomousProcedure procedure) {
        procedureSteps.add(new ProcedureStep((prevState) ->  {
            procedure.startingStepStatus = prevState;
            procedure.run();
            return procedure.lastStepStatus;
        }, true));

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

    /**
     * Adds all steps of another procedure to this one. This function ensures
     * that the enclosed procedure will operate the same as though its steps
     * were individually added to this procedure.
     */
    public AutonomousProcedure then(AutonomousProcedure procedure) {
        procedureSteps.add(new ProcedureStep((prevState) ->  {
            procedure.startingStepStatus = prevState;
            procedure.run();
            return procedure.lastStepStatus;
        }, false));

        return this;
    }

    @Override
    public void run() {
        StepStatus prevState = startingStepStatus;

        for (ProcedureStep step : procedureSteps) {
            prevState = step.run(prevState);
        }

        lastStepStatus = prevState;
    }

    @Override
    public void print() {
        SmartDashboard.putNumber("Autonomous Procedure (" + name + ") step number", procedureStep);
    }

    /**
     * If the given function does not indicate completion by the given time in 
     * seconds, indicate completion anyways.
     */
    public static Function<StepStatus, StepStatus> timeoutAt(double timeoutSeconds, Function<StepStatus, StepStatus> fn) {
        StatusedTimer timer = new StatusedTimer();

        return (prevStatus) -> {
            if (!timer.getStatus()) {
                timer.start();
            } else if (timer.get() >= timeoutSeconds) {
                return StepStatus.Done;
            }

            return fn.apply(prevStatus);
        };
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

            prevState = fn.apply(previousStep);
            return prevState;
        }
    }

    public enum StepStatus {
        Waiting,
        Running,
        Done
    }
}
