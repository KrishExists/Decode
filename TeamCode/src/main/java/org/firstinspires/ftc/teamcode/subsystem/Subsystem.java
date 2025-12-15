package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;

public interface Subsystem {
    public void init();
    public void update(Gamepad gamepad1, Gamepad gamepad2);
}
