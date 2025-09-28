package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class BallShooterProto extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = 24; // -1 means detect any tag
    private VisionPortal visionPortal;
    double drive = 0, strafe = 0, turn = 0;
    private boolean targetFound;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "armMotorLeft");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "armMotorRight");
       // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        double speed = 0.0;
        initAprilTag();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            targetFound = false;
            desiredTag = null;
            boolean obelisk = false;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if(detection.id == 23 || detection.id == 21 ||detection.id == 22 )
                        obelisk = true;
                    if ((detection.id == 24)) {
                        targetFound = true;
                        desiredTag = detection;
                        break;
                    } else {
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            if (targetFound) {
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            } else {
                telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            }
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            if(gamepad1.dpadDownWasReleased()){
                if(speed<0.05){

                }else{
                    speed-=0.05;
                }
            }
            if(gamepad1.dpadUpWasReleased()){
                if(speed>0.096){

                }else{
                    speed+=0.05;
                }
            }
            if(gamepad1.dpadLeftWasReleased()){
                if(speed>0.91){

                }else{
                    speed +=0.1;
                }
            }
            if(gamepad1.rightBumperWasReleased()){
                speed +=0.01;
            }
            if(gamepad1.leftBumperWasReleased()){
                speed -=0.01;
            }
            if(gamepad1.dpadRightWasReleased()){
                if(speed<0.09){

                }else{
                    speed -=0.1;
                }
            }
            if(gamepad1.aWasReleased()){
                speed = 0.6;
            }
            if(gamepad1.bWasReleased()){
                speed = 0;
            }
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            shooter2.setPower(speed);
            shooter1.setPower(speed);
            telemetry.addData("Speed: ", speed);
            telemetry.addData("Current Motor 1 Pos: ", (shooter1.getVelocity()/28) * 60);
            telemetry.addData("Current Motor 2 Pos: ", (shooter2.getVelocity()/28) *60);
            double motor1pos = shooter1.getVelocity()/28 * 60;
            double motor2pos = shooter2.getVelocity()/28 * 60;
            if(Math.abs(motor1pos-motor2pos)<100){
                telemetry.addData("Equal", "true");
            }
            else{
                telemetry.addData("Equal", "false");
            }
            telemetry.update();
        }
    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
}
