package org.firstinspires.ftc.teamcode.PedroHelper;

public interface Step {
    /** Called exactly once when this step becomes active. */
    void onEnter();

    /** Called every update() tick while this step is active. */
    void execute();

    /** Return true to advance to the next step. */
    boolean isComplete();
}