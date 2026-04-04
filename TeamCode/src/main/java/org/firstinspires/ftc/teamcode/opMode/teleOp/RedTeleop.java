

package org.firstinspires.ftc.teamcode.opMode.teleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
// this is testing

@TeleOp(name = "RedTeleop", group = "Main")
public class RedTeleop extends LinearOpMode {
    private HardwareMap hw;
    private Drivetrain drive;
    private Outtake shooter;
    private Intake intake;

    private Robot robot;
    private Turret turret;

    @Override
    public void runOpMode() throws InterruptedException {


// ===== Initialize Hardware & Subsystems =====
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hw = hardwareMap;
        shooter = new Outtake(hw, telemetry);
        drive = new Drivetrain(hardwareMap, telemetry,true);
        Follower follower  = drive.returnFollwer();
        intake = new Intake(hw, telemetry, shooter,follower,true);
        Turret turret = new Turret(hw,telemetry,follower,true);
        robot = new Robot(hw,telemetry,drive,intake,turret, null);
        telemetry.addLine("Initialized — Waiting for Start");
        telemetry.update();
        robot.init();
        waitForStart();
        if (isStopRequested()) return;


// ===== Main Loop =====
        while (opModeIsActive()) {
            robot.update(gamepad1,gamepad2);
            telemetry.update();

        }
    }


}
