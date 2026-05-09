package org.firstinspires.ftc.teamcode.StateMachine;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Wraps an FTC Gamepad and exposes every input as a Button.
 * Call update() once per loop tick before sm.updateAll().
 *
 * Usage:
 *   Buttons gp1 = new Buttons(gamepad1);
 *   Buttons gp2 = new Buttons(gamepad2);
 *
 *   // in loop:
 *   gp1.update();
 *   gp2.update();
 *   sm.updateAll();
 *
 *   // in addState:
 *   .nextOn(gp2.y::pressed,          "ClawClose")
 *   .nextOn(gp2.a::released,         "Rest")
 *   .nextOn(() -> gp2.b.held(0.5),   "EmergencyStop")
 *   .nextOn(gp2.x::doublePress,      "ToggleMode")
 *   .nextOn(gp1.lb::pressed,         "ArmUp")
 *
 * Analog sticks and triggers are exposed as raw floats via leftStickX(),
 * leftStickY(), rightStickX(), rightStickY(), leftTrigger(), rightTrigger().
 * Wrap them in a lambda if you need them as a BooleanSupplier:
 *   .nextOn(() -> gp1.leftTrigger() > 0.5, "Intake")
 */
public class Buttons {

    private final Gamepad gamepad;

    // face buttons
    public final Button a;
    public final Button b;
    public final Button x;
    public final Button y;

    // bumpers
    public final Button lb;
    public final Button rb;

    // triggers as buttons (threshold 0.5)
    public final Button lt;
    public final Button rt;

    // dpad
    public final Button dpadUp;
    public final Button dpadDown;
    public final Button dpadLeft;
    public final Button dpadRight;

    // stick clicks
    public final Button leftStickButton;
    public final Button rightStickButton;

    // start / back / guide
    public final Button start;
    public final Button back;

    public Buttons(Gamepad gamepad) {
        this.gamepad = gamepad;

        a = new Button(() -> gamepad.a);
        b = new Button(() -> gamepad.b);
        x = new Button(() -> gamepad.x);
        y = new Button(() -> gamepad.y);

        lb = new Button(() -> gamepad.left_bumper);
        rb = new Button(() -> gamepad.right_bumper);

        lt = new Button(() -> gamepad.left_trigger  > 0.2f);
        rt = new Button(() -> gamepad.right_trigger > 0.2f);

        dpadUp    = new Button(() -> gamepad.dpad_up);
        dpadDown  = new Button(() -> gamepad.dpad_down);
        dpadLeft  = new Button(() -> gamepad.dpad_left);
        dpadRight = new Button(() -> gamepad.dpad_right);

        leftStickButton  = new Button(() -> gamepad.left_stick_button);
        rightStickButton = new Button(() -> gamepad.right_stick_button);

        start = new Button(() -> gamepad.start);
        back  = new Button(() -> gamepad.back);
    }

    /**
     * Call once per loop tick, before sm.updateAll().
     */
    public void update() {
        a.update();
        b.update();
        x.update();
        y.update();

        lb.update();
        rb.update();
        lt.update();
        rt.update();

        dpadUp.update();
        dpadDown.update();
        dpadLeft.update();
        dpadRight.update();

        leftStickButton.update();
        rightStickButton.update();

        start.update();
        back.update();
    }

    // ── analog passthrough ────────────────────────────────────────────

    public float leftStickX()  { return gamepad.left_stick_x;  }
    public float leftStickY()  { return gamepad.left_stick_y;  }
    public float rightStickX() { return gamepad.right_stick_x; }
    public float rightStickY() { return gamepad.right_stick_y; }
    public float leftTrigger() { return gamepad.left_trigger;  }
    public float rightTrigger(){ return gamepad.right_trigger; }
}