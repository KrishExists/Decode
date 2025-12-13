//package org.firstinspires.ftc.teamcode.opMode.teleOp;
//
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.subsystem.ColorSensor;
//import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
//import org.firstinspires.ftc.teamcode.subsystem.Intake;
//import org.firstinspires.ftc.teamcode.subsystem.Outtake;
//import org.firstinspires.ftc.teamcode.subsystem.Vision;
//
//
//@TeleOp(name = "MainTeleDuplicate", group = "Main")
//public class MainTeleDuplicate extends LinearOpMode {
//
//
//    private Hardware hw;
//    private Drivetrain drive;
//    private Outtake shooter;
//    private Intake intake;
//    private ColorSensor colorSensor;
//    //private Vision vision;
//    private boolean starts = true;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//// ===== Initialize Hardware & Subsystems =====
//        hw = new Hardware(hardwareMap);
//        shooter = new Outtake(hardwareMap, telemetry);
//        drive = new Drivetrain(hardwareMap, telemetry);
//        colorSensor = new ColorSensor(hardwareMap);
//        intake = new Intake(hardwareMap, telemetry, shooter,colorSensor);
//      //  vision = new Vision(hw);
//
//
//        telemetry.addLine("Initialized — Waiting for Start");
//        telemetry.update();
//        waitForStart();
//        if (isStopRequested()) return;
//        intake.timerStart();
//        intake.timerReset();
//
//
//// ===== Main Loop =====
//        while (opModeIsActive()) {
////            if(starts){
////                intake.resetTime();
////                starts = false;
////            }//check if this is needed and works
//// Intake State Machine Update
//            intake.update(gamepad2);
//
//
//// Drive: Auto Align OR Manual
//            if (gamepad2.left_bumper) {
//              //  vision.autoAlignToTag(drive);
//            } else {
//                drive.manualDrive(gamepad1);
//            }
//
//            if (gamepad1.a) {
//                drive.alignToPose(new Pose2d(144, 144, Math.toRadians(90)));
//            } else {
//                drive.manualDrive(gamepad1);
//            }
//
//
//
//// Telemetry
//            telemetry.addData("Intake State", intake.getState());
//            telemetry.addData("Shooter RPM", shooter.atSpeed(0, 99999));
//            telemetry.update();
//        }
//    }
//}

package org.firstinspires.ftc.teamcode.opMode.teleOp;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import com.qualcomm.robotcore.util.Range;

import java.util.List;


@TeleOp(name = "MainTeleDuplicate", group = "Main")
public class MainTeleDuplicate extends LinearOpMode {


    private Hardware hw;
    // ===== AprilTag Vision =====
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private static final int DESIRED_TAG_ID = -1;
    private static final double DESIRED_DISTANCE = 12.0;
    private static final double SPEED_GAIN = 0.02;
    private static final double TURN_GAIN = 0.01;

    private Drivetrain drive;
    private Outtake shooter;
    private Intake intake;
    private ColorSensor colorSensor;
    //private Vision vision;
    private boolean starts = true;


    @Override
    public void runOpMode() throws InterruptedException {


// ===== Initialize Hardware & Subsystems =====
        hw = new Hardware(hardwareMap);
        shooter = new Outtake(hardwareMap, telemetry);
        drive = new Drivetrain(hardwareMap, telemetry);
        colorSensor = new ColorSensor(hardwareMap);
        intake = new Intake(hardwareMap, telemetry, shooter,colorSensor);
        //  vision = new Vision(hw);
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();


        telemetry.addLine("Initialized — Waiting for Start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        intake.timerStart();
        intake.timerReset();


// ===== Main Loop =====
        while (opModeIsActive()) {
//            if(starts){
//                intake.resetTime();
//                starts = false;
//            }//check if this is needed and works
// Intake State Machine Update
            intake.update(gamepad2);


// Drive: Auto Align OR Manual
            if (gamepad2.left_bumper) {

                AprilTagDetection tag = null;
                List<AprilTagDetection> detections = aprilTag.getDetections();

                for (AprilTagDetection detection : detections) {
                    if (detection.metadata != null &&
                            (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID)) {
                        tag = detection;
                        break;
                    }
                }

                if (tag != null) {
                    double rangeError = tag.ftcPose.range - DESIRED_DISTANCE;
                    double headingError = tag.ftcPose.bearing;

                    double drivePower = Range.clip(rangeError * SPEED_GAIN, -0.5, 0.5);
                    double turnPower  = Range.clip(headingError * TURN_GAIN, -0.3, 0.3);

                    // IMPORTANT: this assumes your Drivetrain has a drive method
                    drive.moveRobot(drivePower, 0, turnPower);

                    telemetry.addData("AprilTag", "ID %d", tag.id);
                    telemetry.addData("Range", "%.1f", tag.ftcPose.range);
                    telemetry.addData("Bearing", "%.1f", tag.ftcPose.bearing);
                } else {
                    drive.manualDrive(gamepad1);
                    telemetry.addLine("AprilTag: NOT FOUND");
                }

            } else {
                drive.manualDrive(gamepad1);
            }

            if (gamepad1.a) {
                drive.alignToPose(new Pose2d(144, 144, Math.toRadians(90)));
            } else {
                drive.manualDrive(gamepad1);
            }
            if (visionPortal != null) {
                visionPortal.close();
            }




// Telemetry
            telemetry.addData("Intake State", intake.getState());
            telemetry.addData("Shooter RPM", shooter.atSpeed(0, 99999));
            telemetry.update();
        }
    }
}
