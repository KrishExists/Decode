

package org.firstinspires.ftc.teamcode.opMode.teleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;


@TeleOp(name = "TeleOpNewRed", group = "Main")
public class TeleOpNewRed extends LinearOpMode {
    private HardwareMap hw;
    private Drivetrain drive;
    private Outtake shooter;
    private Intake intake;

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {


// ===== Initialize Hardware & Subsystems =====
        hw = hardwareMap;
        shooter = new Outtake(hw, telemetry);
        drive = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hw, telemetry, shooter);
        robot = new Robot(hw,telemetry,drive,intake);
        telemetry.addLine("Initialized — Waiting for Start");
        telemetry.update();
        robot.init();
        waitForStart();
        if (isStopRequested()) return;


// ===== Main Loop =====
        while (opModeIsActive()) {
            robot.update(gamepad1,gamepad2);
            telemetry.addData("Intake State", intake.getState());
            telemetry.addData("Shooter RPM", shooter.getRPM());
            telemetry.update();



        }
    }


}
