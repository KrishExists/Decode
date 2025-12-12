package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;

public interface Subsystem {
    public void update ();
    public void update(Gamepad gamepad2);
}
