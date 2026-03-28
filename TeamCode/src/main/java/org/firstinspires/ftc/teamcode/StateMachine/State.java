package org.firstinspires.ftc.teamcode.StateMachine;

import java.util.*;

public class State {

    public final String name;

    public StateType type = StateType.CONTINUOUS;
    public double durationSeconds = 0.0;

    public Runnable onStart = () -> {};
    public Runnable onUpdate = () -> {};
    public Runnable onStop = () -> {};

    public String next = null;
    // in State.java
    public final List<Double> updateTimestamps = new ArrayList<>();
    public final List<Runnable> updateRunnables = new ArrayList<>();

    public final Map<String, String> nextOtherChannels = new HashMap<>();
    public final List<Transition> transitions = new ArrayList<>();

    private long startTimeMs = 0;
    private boolean started = false;
    boolean crossChannelFired = false;  // fix: only fire cross-channel once per entry

    public State(String name) {
        this.name = name;
    }

    public void start() {
        started = true;
        startTimeMs = System.currentTimeMillis();
        crossChannelFired = false;
        onStart.run();

        for (Transition t : transitions) {
            t.last = false;
        }
    }

    public void update() {
        if (!started) return;

        long elapsed = System.currentTimeMillis() - startTimeMs;

        Runnable active = onUpdate;
        for (int i = 0; i < updateTimestamps.size(); i++) {
            if (elapsed >= (long)(updateTimestamps.get(i) * 1000)) {
                active = updateRunnables.get(i);
            }
        }

        active.run();
    }

    public void stop() {
        if (started) {
            onStop.run();
            started = false;
        }
    }

    public boolean isTimedDone() {
        if (type != StateType.TIMED) return false;
        return (System.currentTimeMillis() - startTimeMs) >= (long)(durationSeconds * 1000);
    }

    public static class Transition {
        public final java.util.function.BooleanSupplier condition;
        public final String nextState;
        public boolean last = false;

        public Transition(java.util.function.BooleanSupplier condition, String nextState) {
            this.condition = condition;
            this.nextState = nextState;
        }
    }
}