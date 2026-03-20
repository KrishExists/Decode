package org.firstinspires.ftc.teamcode.StateMachine;

import java.util.function.BooleanSupplier;

public class ConditionalTransition {
    public final String nextState;
    public final BooleanSupplier condition;

    public ConditionalTransition(String nextState, BooleanSupplier condition) {
        this.nextState = nextState;
        this.condition = condition;
    }
}