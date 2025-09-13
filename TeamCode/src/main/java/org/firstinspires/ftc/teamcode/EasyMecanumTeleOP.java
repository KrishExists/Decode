

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "easyTele", group = "Concept")
public class EasyMecanumTeleOP extends OpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Vision
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final boolean USE_WEBCAM = true;

    // Drive mode
    private boolean driveForward = true;

    // Tag metadata
    private String motif;

    @Override
    public void init() {
        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initAprilTag();

        telemetry.addLine("DS preview on/off: 3 dots > Camera Stream");
        telemetry.addLine("Press START to begin");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetryAprilTag();

        // Toggle drive direction
        if (gamepad1.dpad_down) {
            driveForward = !driveForward;
        }

        // Auto-align using AprilTag yaw when 'A' is pressed
        if (gamepad1.a) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);

                if (tag.ftcPose != null) {
                    double yaw = tag.ftcPose.yaw;  // Degrees
                    double kRotate = 0.01;
                    double rotateCorrection = -yaw * kRotate;

                    // Deadband and clamp
                    if (Math.abs(yaw) < 1.0) {
                        rotateCorrection = 0;
                    }
                    rotateCorrection = Math.max(-0.4, Math.min(0.4, rotateCorrection));

                    // Rotate only
                    double fl = rotateCorrection;
                    double bl = rotateCorrection;
                    double fr = -rotateCorrection;
                    double br = -rotateCorrection;

                    frontLeft.setPower(fl);
                    backLeft.setPower(bl);
                    frontRight.setPower(fr);
                    backRight.setPower(br);

                    telemetry.addLine("Auto-aligning using yaw...");
                    telemetry.addData("Yaw (deg)", yaw);
                    telemetry.addData("Rotate Correction", rotateCorrection);
                }
            } else {
                telemetry.addLine("No AprilTag detected.");
            }

        } else {
            // Standard mecanum drive
            double y = driveForward ? -gamepad1.left_stick_y : gamepad1.left_stick_y;
            double x = (driveForward ? 1 : -1) * gamepad1.left_stick_x * 1.1;
            double rx = (driveForward ? 1 : -1) * gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }

        telemetry.addData("Drive Direction", driveForward ? "Forward" : "Reverse");
        telemetry.update();
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# Tags Detected", detections.size());

        for (AprilTagDetection tag : detections) {
            if (tag.metadata != null) {
                telemetry.addLine(String.format("ID %d: %s", tag.id, tag.metadata.name));
                telemetry.addLine(String.format("XYZ: (%.1f, %.1f, %.1f)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                telemetry.addLine(String.format("YAW: %.1f deg", tag.ftcPose.yaw));

                if (tag.id == 21) motif = "gpp";
                if (tag.id == 22) motif = "pgp";
                if (tag.id == 23) motif = "ppg";
            } else {
                telemetry.addLine(String.format("ID %d: Unknown", tag.id));
            }
        }

        telemetry.addData("Motif", motif);
    }
}
