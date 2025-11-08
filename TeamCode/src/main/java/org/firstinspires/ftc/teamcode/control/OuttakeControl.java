package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

public class OuttakeControl implements Control {

    private Outtake outtake;
    private Gamepad gp1;

    private Intake intake;

    private Telemetry telemetry;

    private DosaController dosaController;
    private Gamepad gp2;

//    public OuttakeControl(Intake intake, Outtake outtake, Gamepad gp1, Gamepad gp2) {
//        this.outtake = outtake;
//        this.intake = intake;
//        this.gp1 = gp1;
//        this.gp2 = gp2;
//        dosaController = new DosaController(intake, outtake, telemetry);
//    }

    public OuttakeControl(Robot robot, Gamepad gp1, Gamepad gp2) {
       // this(robot.outtake, gp1, gp2);
    }

    @Override
    public void update() {
        // TODO: add controls because ur shi is too confusing for me to rewrite
        if (gp2.left_trigger > 0.1) dosaController.setState(DosaController.State.OUTTAKE);
    }
}
