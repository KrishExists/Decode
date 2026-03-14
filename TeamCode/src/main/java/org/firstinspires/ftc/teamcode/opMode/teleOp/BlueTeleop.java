

package org.firstinspires.ftc.teamcode.opMode.teleOp;


import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Turret;


@TeleOp(name = "BlueTeleop", group = "Main")
public class BlueTeleop extends LinearOpMode {
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
        drive = new Drivetrain(hardwareMap, telemetry,false);
        Follower follower  = drive.returnFollwer();
        intake = new Intake(hw, telemetry, shooter,follower,false);
        Turret turret = new Turret(hw,telemetry,follower,false);
        robot = new Robot(hw,telemetry,drive,intake,turret);
        robot.init();
        telemetry.addLine("Initialized — Waiting for Start");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;


// ===== Main Loop =====
        while (opModeIsActive()) {
            robot.update(gamepad1,gamepad2);
            telemetry.update();



        }
    }


}
