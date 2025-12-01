package org.firstinspires.ftc.teamcode.util;


public class StateMachine<T extends Enum<T>> {
    private T current;
    private T previous;
    private long stateStartMs;


    public StateMachine(T initial) {
        current = initial;
        previous = initial;
        stateStartMs = System.currentTimeMillis();
    }


    public void setState(T next) {
        if (next != current) {
            previous = current;
            current = next;
            stateStartMs = System.currentTimeMillis();
        }
    }


    public T getState() { return current; }
    public T getPrevious() { return previous; }
    public long timeInStateMs() { return System.currentTimeMillis() - stateStartMs; }
}