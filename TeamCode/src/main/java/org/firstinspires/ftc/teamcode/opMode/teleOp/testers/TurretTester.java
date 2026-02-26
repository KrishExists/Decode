package org.firstinspires.ftc.teamcode.opMode.teleOp.testers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@TeleOp(name = "Turret Tester", group = "Testers")
@Config
public class TurretTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


// ===== Initialize Hardware & Subsystems =====
        Drivetrain drivetrain = new Drivetrain(hardwareMap,telemetry);
        Follower follower = drivetrain.returnFollwer();
        Turret turret = new Turret(hardwareMap,telemetry,follower);
        telemetry.addLine("Initialized — Waiting for Start");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;


// ===== Main Loop =====
        while (opModeIsActive()) {
            drivetrain.update(gamepad1,gamepad2);
            turret.updatetelem(gamepad1,gamepad2);
            telemetry.update();

        }
    }
}
