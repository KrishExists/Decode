package org.firstinspires.ftc.teamcode.StateMachine;

import java.util.function.BooleanSupplier;

public class StateBuilder {

    private final State state;

    public StateBuilder(State state) {
        this.state = state;
    }

    public StateBuilder continuous() {
        state.type = StateType.CONTINUOUS;
        return this;
    }

    public StateBuilder timed(double seconds) {
        state.type = StateType.TIMED;
        state.durationSeconds = seconds;
        return this;
    }


    public StateBuilder onStart(Runnable r) {
        state.onStart = r != null ? r : () -> {};
        return this;
    }

    // existing
    public StateBuilder onUpdate(Runnable r) {
        state.onUpdate = r;
        return this;
    }

    // new overload — same method name, different parameters
    public StateBuilder onUpdate(double seconds, Runnable r) {
        state.updateTimestamps.add(seconds);
        state.updateRunnables.add(r);
        return this;
    }

    public StateBuilder onStop(Runnable r) {
        state.onStop = r != null ? r : () -> {};
        return this;
    }

    public StateBuilder next(String nextState) {
        state.next = nextState;
        return this;
    }

    // how to transtionto next stuff
    public StateBuilder nextOn(BooleanSupplier condition, String nextState) {
        state.transitions.add(new State.Transition(condition, nextState));
        return this;
    }


    public StateBuilder nextOtherChannel(String channel, String targetState) {
        state.nextOtherChannels.put(channel, targetState);
        return this;
    }// work on this

    public State build() {
        return state;
    }
}