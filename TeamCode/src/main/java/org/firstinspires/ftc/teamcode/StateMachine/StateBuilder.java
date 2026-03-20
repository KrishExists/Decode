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

    public StateBuilder chained() {
        state.type = StateType.CHAINED;
        return this;
    }

    public StateBuilder onStart(Runnable r) {
        state.onStart = r != null ? r : () -> {};
        return this;
    }

    public StateBuilder onUpdate(Runnable r) {
        state.onUpdate = r != null ? r : () -> {};
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

    // unified transition (buttons, triggers, sensors, anything)
    public StateBuilder nextOnButton(BooleanSupplier condition, String nextState) {
        state.transitions.add(new State.Transition(condition, nextState));
        return this;
    }

    // alias for readability
    public StateBuilder nextIf(BooleanSupplier condition, String nextState) {
        return nextOnButton(condition, nextState);
    }

    public StateBuilder nextOtherChannel(String channel, String targetState) {
        state.nextOtherChannels.put(channel, targetState);
        return this;
    }

    public State build() {
        return state;
    }
}