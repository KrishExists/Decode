package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;

public interface Subsystem {
    void init();

    //public void update ();
    public void update(Gamepad gamepad2);
}
