package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

public class IntakeControl implements Control {

    private Intake intake;
    private Gamepad gp1, gp2;

    public IntakeControl(Intake intake, Gamepad gp1, Gamepad gp2) {
        this.intake = intake;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public IntakeControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.intake, gp1, gp2);
    }

    @Override
    public void update() {
        double power = gp2.right_trigger - (gp2.right_bumper ? 1 : 0);
        intake.setPower(power);
    }
}
