package org.firstinspires.ftc.teamcode.StateMachine;

import java.util.*;
import java.util.function.Function;

public class StateMachine {

    private final Map<String, Map<String, State>> statesByChannel = new LinkedHashMap<>();
    private final Map<String, State> activeStates = new LinkedHashMap<>();


    public StateMachine addState(String name, String channel, Function<StateBuilder, StateBuilder> config) {
        State s = new State(name);
        StateBuilder builder = new StateBuilder(s);
        config.apply(builder).build();

        statesByChannel
                .computeIfAbsent(channel, c -> new LinkedHashMap<>())
                .put(name, s);

        return this;
    }


    public StateMachine validate() {
        for (Map.Entry<String, Map<String, State>> ch : statesByChannel.entrySet()) {
            String channel = ch.getKey();
            Map<String, State> states = ch.getValue();

            for (State s : states.values()) {
                if (s.next != null && !states.containsKey(s.next))
                    throw new RuntimeException(
                            "[StateMachine] " + channel + "/" + s.name
                                    + ": .next(\"" + s.next + "\") not found in channel");

                for (State.Transition t : s.transitions)
                    if (!states.containsKey(t.nextState))
                        throw new RuntimeException(
                                "[StateMachine] " + channel + "/" + s.name
                                        + ": transition target \"" + t.nextState + "\" not found in channel");

                for (Map.Entry<String, String> x : s.nextOtherChannels.entrySet()) {
                    Map<String, State> other = statesByChannel.get(x.getKey());
                    if (other == null)
                        throw new RuntimeException(
                                "[StateMachine] " + channel + "/" + s.name
                                        + ": cross-channel target channel \"" + x.getKey() + "\" not found");
                    if (!other.containsKey(x.getValue()))
                        throw new RuntimeException(
                                "[StateMachine] " + channel + "/" + s.name
                                        + ": cross-channel target state \"" + x.getKey() + "/" + x.getValue() + "\" not found");
                }
            }
        }
        return this;
    }

    // ===== INIT =====

    /**
     * Set the starting state for a channel. Safe to call multiple times —
     * will cleanly stop the current state before starting the new one.
     */
    public void setInitialState(String channel, String stateName) {
        // fix: stop existing state if channel is already active
        State existing = activeStates.get(channel);
        if (existing != null) existing.stop();

        State s = getState(channel, stateName);
        if (s == null) throw new RuntimeException(
                "[StateMachine] setInitialState: \"" + stateName + "\" not found in channel \"" + channel + "\"");

        activeStates.put(channel, s);
        s.start();
    }

    // ===== RUNTIME =====

    /** Call this every loop iteration (inside opMode.loop() or a while loop). */
    public void updateAll() {
        for (String channel : new ArrayList<>(activeStates.keySet())) {
            updateChannel(channel);
        }
    }

    /** Returns the name of the currently active state on a channel, or null if not initialized. */
    public String getCurrentState(String channel) {
        State s = activeStates.get(channel);
        return s == null ? null : s.name;
    }

    // ===== INTERNALS =====

    private State getState(String channel, String name) {
        Map<String, State> map = statesByChannel.get(channel);
        return map == null ? null : map.get(name);
    }

    private void updateChannel(String channel) {
        State current = activeStates.get(channel);
        if (current == null) return;

        current.update();

        // 1. CONDITION TRANSITIONS — rising edge only
        for (State.Transition t : current.transitions) {
            boolean now = t.condition.getAsBoolean();
            if (now && !t.last) {
                t.last = true;
                switchState(channel, t.nextState);
                return;
            }
            t.last = now;
        }

        // 2. CROSS-CHANNEL — fires once per state entry, not every tick
        if (!current.crossChannelFired && !current.nextOtherChannels.isEmpty()) {
            current.crossChannelFired = true;
            for (Map.Entry<String, String> entry : current.nextOtherChannels.entrySet()) {
                switchState(entry.getKey(), entry.getValue());
            }
        }

        // 3. TIMED — advance after duration
        if (current.type == StateType.TIMED && current.isTimedDone()) {
            if (current.next != null) switchState(channel, current.next);
            return;
        }

        // 4. CHAINED — advance immediately next tick (after one update())
        if (current.type == StateType.CHAINED) {
            if (current.next != null) switchState(channel, current.next);
        }
    }

    private void switchState(String channel, String nextName) {
        Map<String, State> map = statesByChannel.get(channel);
        if (map == null) return;

        State next = map.get(nextName);
        if (next == null) return;

        State current = activeStates.get(channel);
        if (current != null && current.name.equals(next.name)) return;

        if (current != null) current.stop();

        activeStates.put(channel, next);
        next.start();
    }
}