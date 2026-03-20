package org.firstinspires.ftc.teamcode.PedroHelper;

import java.util.function.BooleanSupplier;

// ─────────────────────────────────────────────────────────────────────────────
// WaitMsStep — blocks for a fixed number of milliseconds
// ─────────────────────────────────────────────────────────────────────────────

class WaitMsStep implements Step {

    private final long durationMs;
    private long startTime;

    WaitMsStep(long durationMs) {
        this.durationMs = durationMs;
    }

    @Override public void onEnter()  { startTime = System.currentTimeMillis(); }
    @Override public void execute()  { }
    @Override public boolean isComplete() {
        return System.currentTimeMillis() - startTime >= durationMs;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// WaitUntilStep — blocks until a condition returns true
// ─────────────────────────────────────────────────────────────────────────────

class WaitUntilStep implements Step {

    private final BooleanSupplier condition;

    WaitUntilStep(BooleanSupplier condition) {
        this.condition = condition;
    }

    @Override public void onEnter()  { }
    @Override public void execute()  { }
    @Override public boolean isComplete() { return condition.getAsBoolean(); }
}

// ─────────────────────────────────────────────────────────────────────────────
// RunOnceStep — fires a Runnable once then immediately advances
// ─────────────────────────────────────────────────────────────────────────────

class RunOnceStep implements Step {

    private final Runnable action;
    private boolean ran = false;

    RunOnceStep(Runnable action) {
        this.action = action;
    }

    @Override public void onEnter()  { action.run(); ran = true; }
    @Override public void execute()  { }
    @Override public boolean isComplete() { return ran; }
}

// ─────────────────────────────────────────────────────────────────────────────
// StopStep — halts everything; AutoRoutine checks for this type
// ─────────────────────────────────────────────────────────────────────────────

class StopStep implements Step {

    @Override public void onEnter()  { }
    @Override public void execute()  { }
    @Override public boolean isComplete() { return true; }
}