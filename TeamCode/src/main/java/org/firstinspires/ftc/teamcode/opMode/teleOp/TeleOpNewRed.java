

package org.firstinspires.ftc.teamcode.opMode.teleOp;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;
import java.util.concurrent.TimeUnit;


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
