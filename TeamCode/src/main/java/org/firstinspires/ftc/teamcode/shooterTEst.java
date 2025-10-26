package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "ShooterTEst", group = "Main")
public class shooterTEst extends LinearOpMode {
    final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    VisionPortal visionPortal;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor intake = hardwareMap.dcMotor.get("Intake");
        DcMotor outtake = hardwareMap.dcMotor.get("Outtake");
        Servo linkage = hardwareMap.get(Servo.class, "Linkage");
        double linkagePos = 0;
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double power = 0;
        double intkePower = 1;


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetryAprilTag();
            telemetry.addData("OutakeSpeed",power);
            telemetry.addData("IntakeSpeed",intkePower);

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
            }

            // Share the CPU.
            sleep(20);
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            telemetry.addData("Outake Power",outtake.getPower());
            telemetry.update();
            intake.setPower(intkePower);
            outtake.setPower(power);
            linkage.setPosition(linkagePos);
            if(gamepad1.dpadUpWasPressed()){
                power+=0.1;
            }
            if(gamepad1.dpadDownWasReleased()){
                power-=0.1;
            }
            if(gamepad1.dpadLeftWasPressed()){
                power+=0.05;
            }if(gamepad1.dpadRightWasPressed()){
                power-=0.05;
            }if(gamepad1.rightBumperWasPressed()){
                power+=0.01;
            }if(gamepad1.leftBumperWasReleased()){
                power-=0.01;
            }
            if(gamepad1.aWasPressed()){
                power = 0;
            }if(gamepad1.backWasPressed()){
                power = 0.8;
            }
            //intake
            if(gamepad2.dpadUpWasPressed()){
                intkePower+=0.1;
            }
            if(gamepad2.dpadDownWasReleased()){
                intkePower-=0.1;
            }
            if(gamepad2.dpadLeftWasPressed()){
                intkePower+=0.05;
            }if(gamepad2.dpadRightWasPressed()){
                intkePower-=0.05;
            }if(gamepad2.rightBumperWasPressed()){
                intkePower+=0.01;
            }if(gamepad2.leftBumperWasReleased()){
                intkePower-=0.01;
            }
            if(gamepad2.aWasPressed()){
                intkePower = 0;
            }if(gamepad2.backWasPressed()){
                intkePower = 0.8;
            }
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
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

    }   // end method initAprilTag()
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



                // detection.ftcPose.x




            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}