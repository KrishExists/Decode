

package org.firstinspires.ftc.teamcode.opMode.teleOp;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystem.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Vision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "TeleOpNewRed", group = "Main")
public class TeleOpNewRed extends LinearOpMode {


    private Hardware hw;
    // ===== AprilTag Vision =====



    private static final int DESIRED_TAG_ID = -1;
    private static final double DESIRED_DISTANCE = 12.0;
    private static final double SPEED_GAIN = 0.02;
    private static final double TURN_GAIN = 0.01;
    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_TURN  = 0.3;
    private Drivetrain drive;
    private Outtake shooter;
    private Intake intake;
    private ColorSensor colorSensor;
    //private Vision vision;
    private boolean starts = true;
    private static final boolean USE_WEBCAM = true;

    private AprilTagDetection desiredTag = null;
    private Vision vision;

    @Override
    public void runOpMode() throws InterruptedException {


// ===== Initialize Hardware & Subsystems =====
        hw = new Hardware(hardwareMap);
        shooter = new Outtake(hardwareMap, telemetry);
        drive = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, shooter,colorSensor);
        vision = new Vision(hw);
        boolean targetFound;



        telemetry.addLine("Initialized — Waiting for Start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        intake.timerStart();
        intake.timerReset();


// ===== Main Loop =====
        while (opModeIsActive()) {

            drive.update();
            intake.update(gamepad1, gamepad2);


// Drive: Auto Align OR Manual
            drive.combinedDrive(gamepad1);

            Pose2d currentPOse =drive.getPose();
            AprilTagDetection tag=  vision.detectTag(drive);
// Telemetry
            telemetry.addData("Intake State", intake.getState());
            telemetry.addData("Pose position",currentPOse.position);
            telemetry.addData("Pose heading",currentPOse.heading);
            telemetry.addData("Shooter RPM", shooter.atSpeed(0, 99999));
            telemetry.addData("Tag Distance", tag.ftcPose.range);

        }
    }

}
