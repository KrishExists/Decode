

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

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "TeleOpNewRed", group = "Main")
public class TeleOpNewRed extends LinearOpMode {


    private Hardware hw;
    // ===== AprilTag Vision =====



    private static final int DESIRED_TAG_ID = -1;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
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

    private AprilTagDetection desiredTag = null;
    private Vision vision;

    @Override
    public void runOpMode() throws InterruptedException {


// ===== Initialize Hardware & Subsystems =====
        hw = new Hardware(hardwareMap);
        shooter = new Outtake(hardwareMap, telemetry);
        drive = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, shooter,colorSensor);

        boolean targetFound;



        telemetry.addLine("Initialized — Waiting for Start");
        telemetry.update();
        initAprilTag();
        waitForStart();
        if (isStopRequested()) return;
        intake.timerStart();
        intake.timerReset();


// ===== Main Loop =====
        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            AprilTagDetection detected = null;
            for (AprilTagDetection detection : currentDetections) {
                if(detection.id==20||detection.id ==24){
                    detected = detection;
                    break;
                }
            }

            drive.update();
            intake.update(gamepad1, gamepad2);


// Drive: Auto Align OR Manual
            if(detected!=null){
                drive.combinedDrive(gamepad1, detected);
                telemetry.addData("Detected tag ", 1);


            }else{
                telemetry.addData("Detected tag", 0);

                drive.combinedDrive(gamepad1);
            }

            Pose2d currentPOse =drive.getPose();
            
// Telemetry
            telemetry.addData("Intake State", intake.getState());
            telemetry.addData("Pose position",currentPOse.position);
            telemetry.addData("Pose heading",currentPOse.heading);
            telemetry.addData("Shooter RPM", shooter.atSpeed(0, 99999));
            telemetry.update();



        }
    }
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);


    }   // end method initAprilTag()


}
