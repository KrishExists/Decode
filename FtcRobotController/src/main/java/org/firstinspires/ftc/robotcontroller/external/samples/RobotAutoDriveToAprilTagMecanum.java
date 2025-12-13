package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Mecanum Drive To AprilTag", group = "Concept")
//@Disabled
public class RobotAutoDriveToAprilTagMecanum extends LinearOpMode {

    // ================== TUNING ==================
    final double DESIRED_DISTANCE = 12.0; // inches
    final double SPEED_GAIN = 0.02;
    final double TURN_GAIN = 0.01;

    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_TURN  = 0.3;

    // ================== MOTORS ==================
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ================== VISION ==================
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1; // -1 = any tag

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    @Override
    public void runOpMode() {

        boolean targetFound;
        double drive, strafe, turn;

        initAprilTag();

        // Hardware mapping
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left");
        backRight  = hardwareMap.get(DcMotor.class, "back_right");

        // Motor directions (typical mecanum)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }

        telemetry.addData(">", "Press START");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            targetFound = false;
            desiredTag = null;

            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    if (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID) {
                        targetFound = true;
                        desiredTag = detection;
                        break;
                    }
                }
            }



            if (gamepad1.left_bumper && targetFound) {
                // AUTO MODE
                double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                double headingError = desiredTag.ftcPose.bearing;

                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = 0;

                telemetry.addData("AUTO", "Drive %.2f Turn %.2f", drive, turn);
            } else {
                // MANUAL POV DRIVE
                drive  = -gamepad1.left_stick_y * 0.6;
                strafe =  gamepad1.left_stick_x * 0.6;
                turn   = -gamepad1.right_stick_x * 0.5;

                telemetry.addData("MANUAL", "D %.2f S %.2f T %.2f", drive, strafe, turn);
            }

            telemetry.update();
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    // ================== MECANUM DRIVE ==================
    private void moveRobot(double drive, double strafe, double turn) {

        double fl = drive + strafe + turn;
        double fr = drive - strafe - turn;
        double bl = drive - strafe + turn;
        double br = drive + strafe - turn;

        double max = Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        );

        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    // ================== APRILTAG INIT ==================
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
