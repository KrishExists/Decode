package org.firstinspires.ftc.teamcode.opMode.teleOp.testers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@TeleOp(name = "Turret Tester", group = "Testers")
@Config
public class TurretTester extends LinearOpMode {
    DcMotor transfer;
    Servo linkage;
    DcMotor intake;

    public static double rpm;
    public static double intakePower;

    public static double transferPower;
    public static double linkagePos;

    @Override
    public void runOpMode() throws InterruptedException {
        rpm = 0;
        // === Hardware Init ===
        linkage = hardwareMap.get(Servo.class, "Linkage");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
        intake = hardwareMap.get(DcMotor.class, "Intake");

        intakePower = 0;
        transferPower = 0;
        linkagePos = 0.5;

// ===== Initialize Hardware & Subsystems =====
        Drivetrain drivetrain = new Drivetrain(hardwareMap,telemetry);
        Outtake outtake = new Outtake(hardwareMap,telemetry);
        Follower follower = drivetrain.returnFollwer();
        Turret turret = new Turret(hardwareMap,telemetry,follower);
        telemetry.addLine("Initialized — Waiting for Start");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;


// ===== Main Loop =====
        while (opModeIsActive()) {
            linkage.setPosition(linkagePos);
            drivetrain.update(gamepad1,gamepad2);
            turret.updatetelem(gamepad1,gamepad2);
            outtake.spinToRpm(rpm);
            intake.setPower(intakePower);
            transfer.setPower(transferPower);
            telemetry.update();

        }
    }
}
