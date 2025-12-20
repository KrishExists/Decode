////package org.firstinspires.ftc.teamcode.opMode.teleOp;
////
////
////import com.acmerobotics.roadrunner.Pose2d;
////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
////import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////import org.firstinspires.ftc.teamcode.Hardware;
////import org.firstinspires.ftc.teamcode.subsystem.ColorSensor;
////import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
////import org.firstinspires.ftc.teamcode.subsystem.Intake;
////import org.firstinspires.ftc.teamcode.subsystem.Outtake;
////import org.firstinspires.ftc.teamcode.subsystem.Vision;
////
////
////@TeleOp(name = "MainTeleDuplicate", group = "Main")
////public class MainTeleDuplicate extends LinearOpMode {
////
////
////    private Hardware hw;
////    private Drivetrain drive;
////    private Outtake shooter;
////    private Intake intake;
////    private ColorSensor colorSensor;
////    //private Vision vision;
////    private boolean starts = true;
////
////
////    @Override
////    public void runOpMode() throws InterruptedException {
////
////
////// ===== Initialize Hardware & Subsystems =====
////        hw = new Hardware(hardwareMap);
////        shooter = new Outtake(hardwareMap, telemetry);
////        drive = new Drivetrain(hardwareMap, telemetry);
////        colorSensor = new ColorSensor(hardwareMap);
////        intake = new Intake(hardwareMap, telemetry, shooter,colorSensor);
////      //  vision = new Vision(hw);
////
////
////        telemetry.addLine("Initialized — Waiting for Start");
////        telemetry.update();
////        waitForStart();
////        if (isStopRequested()) return;
////        intake.timerStart();
////        intake.timerReset();
////
////
////// ===== Main Loop =====
////        while (opModeIsActive()) {
//////            if(starts){
//////                intake.resetTime();
//////                starts = false;
//////            }//check if this is needed and works
////// Intake State Machine Update
////            intake.update(gamepad2);
////
////
////// Drive: Auto Align OR Manual
////            if (gamepad2.left_bumper) {
////              //  vision.autoAlignToTag(drive);
////            } else {
////                drive.manualDrive(gamepad1);
////            }
////
////            if (gamepad1.a) {
////                drive.alignToPose(new Pose2d(144, 144, Math.toRadians(90)));
////            } else {
////                drive.manualDrive(gamepad1);
////            }
////
////
////
////// Telemetry
////            telemetry.addData("Intake State", intake.getState());
////            telemetry.addData("Shooter RPM", shooter.atSpeed(0, 99999));
////            telemetry.update();
////        }
////    }
////}
//
//package org.firstinspires.ftc.teamcode.opMode.teleOp;
//
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.subsystem.ColorSensor;
//import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
//import org.firstinspires.ftc.teamcode.subsystem.Intake;
//import org.firstinspires.ftc.teamcode.subsystem.Outtake;
//import org.firstinspires.ftc.teamcode.subsystem.Vision;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import com.qualcomm.robotcore.util.Range;
//
//import java.util.List;
//import java.util.concurrent.TimeUnit;
//
//
//@TeleOp(name = "MainTeleDuplicate", group = "Main")
//public class MainTeleDuplicate extends LinearOpMode {
//
//
//    private Hardware hw;
//    // ===== AprilTag Vision =====
//    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTag;
//
//
//
//    private static final int DESIRED_TAG_ID = -1;
//    private static final double DESIRED_DISTANCE = 12.0;
//    private static final double SPEED_GAIN = 0.02;
//    private static final double TURN_GAIN = 0.01;
//    final double MAX_AUTO_SPEED = 0.5;
//    final double MAX_AUTO_TURN  = 0.3;
//    private Drivetrain drive;
//    private Outtake shooter;
//    private Intake intake;
//    private ColorSensor colorSensor;
//    //private Vision vision;
//    private boolean starts = true;
//    private static final boolean USE_WEBCAM = true;
//
//    private AprilTagDetection desiredTag = null;
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
//        boolean targetFound;
//
//        initAprilTag();
//
//
//        telemetry.addLine("Initialized — Waiting for Start");
//        telemetry.update();
//        if (USE_WEBCAM) {
//            setManualExposure(6, 250);
//        }
//        waitForStart();
//        if (isStopRequested()) return;
//        intake.timerStart();
//        intake.timerReset();
//
//
//// ===== Main Loop =====
//        while (opModeIsActive()) {
//            targetFound = false;
//            desiredTag = null;
////            if(starts){
////                intake.resetTime();
////                starts = false;
////            }//check if this is needed and works
//// Intake State Machine Update
//            intake.update(gamepad1, gamepad2);
//
//
//// Drive: Auto Align OR Manual
//            if (gamepad1.a) {
//
//                List<AprilTagDetection> detections = aprilTag.getDetections();
//                for (AprilTagDetection detection : detections) {
//                    if (detection.metadata != null) {
//                        if (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID) {
//                            targetFound = true;
//                            desiredTag = detection;
//                            break;
//                        }
//                    }
//                }
//
//                if (targetFound) {
//                    double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
//                    double headingError = desiredTag.ftcPose.bearing;
//
//                   double drive2  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//                   double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//                double    strafe = 0;
//
//                    telemetry.addData("AUTO", "Drive %.2f Turn %.2f", drive, turn);
//                    // IMPORTANT: this assumes your Drivetrain has a drive method
//                    drive.moveRobot(0, 0, turn);
//
//                    telemetry.addData("AprilTag", "ID %d", desiredTag.id);
//                    telemetry.addData("Range", "%.1f", desiredTag.ftcPose.range);
//                    telemetry.addData("Bearing", "%.1f", desiredTag.ftcPose.bearing);
//                    telemetry.addData("AUTO", "Drive %.2f Turn %.2f", drive, turn);
//
//                } else {
//                    drive.manualDrive(gamepad1);
//                    telemetry.addLine("AprilTag: NOT FOUND");
//                }
////end of if gamepad a
//            } else {
//                drive.manualDrive(gamepad1);
//            }
//
//            if (targetFound) {
//                telemetry.addData("Tag", "ID %d", desiredTag.id);
//                telemetry.addData("Range", "%.1f in", desiredTag.ftcPose.range);
//                telemetry.addData("Bearing", "%.1f deg", desiredTag.ftcPose.bearing);
//                telemetry.addData(">", "Hold LEFT BUMPER for auto");
//            } else {
//                telemetry.addData(">", "Find AprilTag");
//            }
//
//            if (visionPortal != null) {
//                visionPortal.close();
//            }
//
//
//
//
//// Telemetry
//            telemetry.addData("Intake State", intake.getState());
//            telemetry.addData("Shooter RPM", shooter.atSpeed(0, 99999));
//            telemetry.update();
//        }
//    }
//    private void initAprilTag() {
//
//        aprilTag = new AprilTagProcessor.Builder().build();
//        aprilTag.setDecimation(2);
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//
//        visionPortal = builder.addProcessor(aprilTag).build();
//    }
//
//    // ================== CAMERA CONTROL ==================
//    private void setManualExposure(int exposureMS, int gain) {
//
//        if (visionPortal == null) return;
//
//        while (!isStopRequested() &&
//                visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            sleep(20);
//        }
//
//        if (!isStopRequested()) {
//            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
//            if (exposure.getMode() != ExposureControl.Mode.Manual) {
//                exposure.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
//            }
//            exposure.setExposure(exposureMS, TimeUnit.MILLISECONDS);
//
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(gain);
//        }
//    }
//}

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystem.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Vision;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import java.util.concurrent.TimeUnit;


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


    @Override
    public void runOpMode() throws InterruptedException {


// ===== Initialize Hardware & Subsystems =====
        hw = new Hardware(hardwareMap);
        shooter = new Outtake(hardwareMap, telemetry);
        drive = new Drivetrain(hardwareMap, telemetry);
        colorSensor = new ColorSensor(hardwareMap);
        intake = new Intake(hardwareMap, telemetry, shooter,colorSensor);
        boolean targetFound;

        initAprilTag();


        telemetry.addLine("Initialized — Waiting for Start");
        telemetry.update();
        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }
        waitForStart();
        if (isStopRequested()) return;
        intake.timerStart();
        intake.timerReset();


// ===== Main Loop =====
        while (opModeIsActive()) {
            targetFound = false;
            desiredTag = null;
//            if(starts){
//                intake.resetTime();
//                starts = false;
//            }//check if this is needed and works
// Intake State Machine Update
            drive.update();
            intake.update(gamepad1, gamepad2);


// Drive: Auto Align OR Manual
           drive.combinedDrive(gamepad1);

            Pose2d currentPOse =drive.getPose();
// Telemetry
            telemetry.addData("Intake State", intake.getState());
            telemetry.addData("Pose position",currentPOse.position);
            telemetry.addData("Pose heading",currentPOse.heading);
            telemetry.addData("Shooter RPM", shooter.atSpeed(0, 99999));

        }
    }
    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        visionPortal = builder.addProcessor(aprilTag).build();
    }

    // ================== CAMERA CONTROL ==================
    private void setManualExposure(int exposureMS, int gain) {

        if (visionPortal == null) return;

        while (!isStopRequested() &&
                visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }

        if (!isStopRequested()) {
            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            if (exposure.getMode() != ExposureControl.Mode.Manual) {
                exposure.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposure.setExposure(exposureMS, TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
        }
    }
}
