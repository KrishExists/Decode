package org.firstinspires.ftc.teamcode.StateMachine;

import java.util.function.BooleanSupplier;

/**
 * Wraps any BooleanSupplier (gamepad button, sensor, etc.) and tracks
 * press/release/hold/double-press state. Call update() once per loop tick.
 *
 * Usage:
 *   Button y = new Button(() -> gamepad2.y);
 *
 *   // in loop:
 *   y.update();
 *
 *   // as nextOn supplier:
 *   .nextOn(y::pressed, "NextState")
 *   .nextOn(y::released, "NextState")
 *   .nextOn(() -> y.held(0.5), "NextState")
 *   .nextOn(() -> y.doublePress(0.4), "NextState")
 */
public class Button {

    private final BooleanSupplier button;

    private boolean current = false;
    private boolean last    = false;

    // held tracking
    private long holdStartMs  = 0;
    private boolean holdFired = false;

    // double press tracking
    private int    pressCount       = 0;
    private long   lastPressTimeMs  = 0;
    private boolean doubleFired     = false;

    public Button(BooleanSupplier source) {
        this.button = source;
    }

    /**
     * Call once per loop tick before reading any state.
     */
    public void update() {
        last    = current;
        current = button.getAsBoolean();

        // hold tracking
        if (current && !last) {
            holdStartMs = System.currentTimeMillis();
            holdFired   = false;
        }
        if (!current) {
            holdFired = false;
        }

        // double press tracking
        if (pressed()) {
            long now = System.currentTimeMillis();
            if (pressCount == 1 && (now - lastPressTimeMs) <= 600) {
                pressCount = 2;
            } else {
                pressCount    = 1;
                lastPressTimeMs = now;
                doubleFired   = false;
            }
        }
    }

    /** True for exactly one tick when the button is first pressed. */
    public boolean pressed() {
        return current && !last;
    }

    /** True for exactly one tick when the button is released. */
    public boolean released() {
        return !current && last;
    }

    /** True while the button is held down. */
    public boolean isDown() {
        return current;
    }
    public boolean isUp() {
        return !current;
    }

    /**
     * True for exactly one tick after the button has been held for the given duration.
     * Resets if the button is released before the duration is reached.
     *
     * @param seconds how long the button must be held
     */
    public boolean held(double seconds) {
        if (!current || holdFired) return false;
        if ((System.currentTimeMillis() - holdStartMs) >= (long)(seconds * 1000)) {
            holdFired = true;
            return true;
        }
        return false;
    }

    /**
     * True for exactly one tick when a double press is detected within the default
     * 600ms window.
     */
    public boolean doublePress() {
        return doublePress(0.6);
    }

    /**
     * True for exactly one tick when two presses occur within windowSeconds.
     *
     * @param windowSeconds max time between presses to count as a double press
     */
    public boolean doublePress(double windowSeconds) {
        if (doubleFired) return false;
        if (pressCount == 2) {
            long now = System.currentTimeMillis();
            if ((now - lastPressTimeMs) <= (long)(windowSeconds * 1000)) {
                doubleFired = true;
                pressCount  = 0;
                return true;
            } else {
                pressCount = 0;
            }
        }
        return false;
    }
}