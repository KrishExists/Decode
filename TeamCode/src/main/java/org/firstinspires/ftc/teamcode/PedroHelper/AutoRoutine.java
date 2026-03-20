package org.firstinspires.ftc.teamcode.PedroHelper;

import com.pedropathing.follower.Follower;

import java.util.List;

public class AutoRoutine {

    private final Follower   follower;
    private final List<Step> steps;

    private int     currentIndex = 0;
    private boolean enteredCurrentStep = false;
    private boolean halted = false;

    AutoRoutine(Follower follower, List<Step> steps) {
        this.follower = follower;
        this.steps    = steps;
    }

    /**
     * Call this every loop() tick. Advances the step queue automatically.
     */
    public void update() {
        if (halted) return;
        if (currentIndex >= steps.size()) return;

        Step current = steps.get(currentIndex);

        // onEnter fires exactly once per step
        if (!enteredCurrentStep) {
            current.onEnter();
            enteredCurrentStep = true;
        }

        current.execute();

        if (current.isComplete()) {
            // StopStep halts — all subsequent steps are skipped
            if (current instanceof StopStep) {
                follower.breakFollowing();
                halted = true;
                return;
            }

            currentIndex++;
            enteredCurrentStep = false;
        }
    }

    /** True once a StopStep is hit or all steps finish. */
    public boolean isFinished() {
        return halted || currentIndex >= steps.size();
    }

    /** Force-stop from outside (e.g. opmode stop()). */
    public void halt() {
        halted = true;
        follower.breakFollowing();
    }

    /** How many steps are in this routine (useful for telemetry). */
    public int totalSteps() { return steps.size(); }

    /** Index of the currently executing step (useful for telemetry). */
    public int currentStep() { return currentIndex; }
}