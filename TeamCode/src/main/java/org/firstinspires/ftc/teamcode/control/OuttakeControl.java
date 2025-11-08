package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

public class OuttakeControl implements Control {

    private Outtake outtake;
    private Gamepad gp1;
    private Gamepad gp2;

    public OuttakeControl(Outtake outtake, Gamepad gp1, Gamepad gp2) {
        this.outtake = outtake;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public OuttakeControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.outtake, gp1, gp2);
    }

    @Override
    public void update() {
        // TODO: add controls because ur shi is too confusing for me to rewrite
    }
}
