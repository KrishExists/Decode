////package org.firstinspires.ftc.teamcode;
////
////import com.qualcomm.robotcore.eventloop.opmode.OpMode;
////import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////import com.qualcomm.robotcore.hardware.DcMotor;
////
////import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
////import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
////import org.firstinspires.ftc.vision.VisionPortal;
////import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
////import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
////
////import java.util.ArrayList;
////import java.util.List;
////
////@TeleOp(name = "MecanumTeleop", group = "Concept")
////public class MecanumTeleOpdeocde extends OpMode {
////
////    // Drive motors
////    private DcMotor frontLeft, frontRight, backLeft, backRight;
////
////    // Vision
////    private AprilTagProcessor aprilTag;
////    private VisionPortal visionPortal;
////    private static final boolean USE_WEBCAM = true;
////
////    // States
////    public boolean autoAlignActive = false;
////    private boolean driveForward = true;
////    private String motif = "unknown"; // initialize
////
////    // Edge detection states
////    private boolean prevAPressed = false;
////    private boolean prevDpadDownPressed = false;
////
////    @Override
////    public void init() {
////        // Initialize motors
////        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
////        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
////        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
////        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
////
////        frontRight.setDirection(DcMotor.Direction.REVERSE);
////        backRight.setDirection(DcMotor.Direction.REVERSE);
////        frontLeft.setDirection(DcMotor.Direction.FORWARD);
////        backLeft.setDirection(DcMotor.Direction.FORWARD);
////
////        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////
////        initAprilTag();
////
////        telemetry.addLine("DS preview on/off: 3 dots > Camera Stream");
////        telemetry.addLine("Press START to begin");
////        telemetry.update();
////    }
////
////    @Override
////    public void loop() {
////        telemetryAprilTag();
////
////        // === RISING EDGE DETECTORS ===
////
////        // A Button → Toggle auto-align
////        boolean aPressed = gamepad1.a;
////        if (aPressed && !prevAPressed) {
////            autoAlignActive = !autoAlignActive;
////        }
////        prevAPressed = aPressed;
////
////        // D-Pad Down → Toggle drive direction
////        boolean dpadDownPressed = gamepad1.dpad_down;
////        if (dpadDownPressed && !prevDpadDownPressed) {
////            driveForward = !driveForward;
////        }
////        prevDpadDownPressed = dpadDownPressed;
////
////        // === AUTO-ALIGNMENT USING YAW ===
////        if (autoAlignActive) {
////            List<AprilTagDetection> detections = aprilTag.getDetections();
////            if (detections == null) detections = new ArrayList<>();
////
////            if (!detections.isEmpty()) {
////                AprilTagDetection tag = detections.get(0);
////
////                if (tag.ftcPose != null) {
////                    double yaw = tag.ftcPose.yaw;  // Degrees
////                    double kRotate = 0.01;
////
////                    // You can tune this value — higher = faster spin
////                    double rotateCorrection = -yaw * kRotate;
////
////                    // Clamp rotation speed
////                    rotateCorrection = Math.max(-0.4, Math.min(0.4, rotateCorrection));
////
////                    // Deadband: Don't spin if we're close enough
////                    if (Math.abs(yaw) < 1.0) {
////                        rotateCorrection = 0;
////                    }
////
////                    // Apply rotation only
////                    // NOTE: because frontRight/backRight are set to REVERSE above,
////                    // the sign you send to them may need flipping depending on real-world behavior.
////                    frontLeft.setPower(rotateCorrection);
////                    backLeft.setPower(rotateCorrection);
////                    frontRight.setPower(-rotateCorrection);
////                    backRight.setPower(-rotateCorrection);
////
////                    telemetry.addLine("Auto-aligning to tag...");
////                    telemetry.addData("Yaw (deg)", yaw);
////                    telemetry.addData("Rotate Correction", rotateCorrection);
////                } else {
////                    telemetry.addLine("Tag detected, but no pose available.");
////                    stopMotors();
////                }
////            } else {
////                telemetry.addLine("No AprilTag detected.");
////                stopMotors();
////            }
////
////        } else {
////            // === STANDARD MANUAL DRIVE ===
////            double y = driveForward ? -gamepad1.left_stick_y : gamepad1.left_stick_y;
////            double x = (driveForward ? 1 : -1) * gamepad1.left_stick_x * 1.1;
////            double rx = (driveForward ? 1 : -1) * gamepad1.right_stick_x;
////
////            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
////            double frontLeftPower = (y + x + rx) / denominator;
////            double backLeftPower = (y - x + rx) / denominator;
////            double frontRightPower = (y - x - rx) / denominator;
////            double backRightPower = (y + x - rx) / denominator;
////
////            frontLeft.setPower(frontLeftPower);
////            backLeft.setPower(backLeftPower);
////            frontRight.setPower(frontRightPower);
////            backRight.setPower(backRightPower);
////        }
////
////        telemetry.addData("Drive Direction", driveForward ? "Forward" : "Reverse");
////        telemetry.addData("Auto Align Active", autoAlignActive);
////        telemetry.update();
////    }
////
////    private void stopMotors() {
////        frontLeft.setPower(0);
////        backLeft.setPower(0);
////        frontRight.setPower(0);
////        backRight.setPower(0);
////    }
////
////    private void initAprilTag() {
////        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
////
////        if (USE_WEBCAM) {
////            visionPortal = VisionPortal.easyCreateWithDefaults(
////                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
////        } else {
////            visionPortal = VisionPortal.easyCreateWithDefaults(
////                    BuiltinCameraDirection.BACK, aprilTag);
////        }
////
////        // If you have issues with the preview not starting, check if visionPortal has a .start() you must call:
////        // try { visionPortal.start(); } catch (Exception e) { /* docs may start it automatically */ }
////    }
////
////    private void telemetryAprilTag() {
////        List<AprilTagDetection> detections = aprilTag.getDetections();
////        if (detections == null) {
////            telemetry.addData("# Tags Detected", 0);
////            telemetry.addData("Motif", motif);
////            return;
////        }
////
////        telemetry.addData("# Tags Detected", detections.size());
////
////        for (AprilTagDetection tag : detections) {
////            if (tag.metadata != null) {
////                telemetry.addLine(String.format("ID %d: %s", tag.id, tag.metadata.name));
////
////                // CHECK ftcPose before using it
////                if (tag.ftcPose != null) {
////                    telemetry.addLine(String.format("XYZ: (%.1f, %.1f, %.1f)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
////                    telemetry.addLine(String.format("YAW: %.1f deg", tag.ftcPose.yaw));
////                } else {
////                    telemetry.addLine("Pose: unavailable");
////                }
////
////                if (tag.id == 21) motif = "gpp";
////                if (tag.id == 22) motif = "pgp";
////                if (tag.id == 23) motif = "ppg";
////            } else {
////                telemetry.addLine(String.format("ID %d: Unknown", tag.id));
////            }
////        }
////
////        telemetry.addData("Motif", motif);
////    }
////}
//
//package org.firstinspires.ftc.teamcode;
//
//import static java.lang.Thread.sleep;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import com.arcrobotics.ftclib.util.InterpLUT;
//
//import java.util.List;
//
//
//@TeleOp(name = "MecanumTeleop", group = "Concept")
//public class MecanumTeleOpdeocde extends OpMode {
//    decodeArm arm;
//    double y;
//    double x;
//    double rx;
//    boolean driveForward;
//    // Motors for mecanum drive
//    private DcMotor frontLeft, frontRight, backLeft, backRight;
//    boolean clawPos = false;
//    boolean slidesExtend = false;  // Toggle flag for slide movement
//    // Arm controller and state machine
//    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//
//    /**
//     * The variable to store our instance of the AprilTag processor.
//     */
//    private AprilTagProcessor aprilTag;
//
//    /**
//     * The variable to store our instance of the vision portal.
//     */
//    private VisionPortal visionPortal;
//    boolean previousYState = false;
//    boolean previousLeftBumperState = false;
//    double clawRotationState = 0;
//    public double pivotPos = 1;
//    double clawOpen = 0.4;
//    private Servo clawServo;
//
//    private Servo pivotServo;
//    public double slidesPos;
//    public String motif;
//
//    public boolean pivotMove = false;
//// 0 backLeftMotor, 1 backRightMotor, 2 frontRightMotor, 3 frontLeftMotor
//
//    @Override
//    public void init() {
//        driveForward = true;
//        // Initialize mecanum drive motors
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
//        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.REVERSE);
//        frontLeft.setDirection(DcMotor.Direction.FORWARD);
//        backLeft.setDirection(DcMotor.Direction.FORWARD);
//
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        initAprilTag();
//
//        // Wait for the DS start button to be touched.
//        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch START to start OpMode");
//        telemetry.update();
//        // Initialize the arm controller and state machine
//
//    }
//
//    @Override
//
//
//    public void loop() {
//        telemetryAprilTag();
//        // Save CPU resources; can resume streaming when needed.
//        if (gamepad1.dpad_left) {
//            visionPortal.stopStreaming();
//        } else if (gamepad1.dpad_up) {
//            visionPortal.resumeStreaming();
//        }
//
//        // Mecanum drive control
//
//        if(driveForward) {
//            y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//            rx = gamepad1.right_stick_x;
//        }else{
//            y = gamepad1.left_stick_y * 1.1;
//            x = -gamepad1.left_stick_x * 1.1;
//            rx = -gamepad1.right_stick_x;
//        }
//        if(gamepad1.dpad_down){
//            driveForward = !driveForward;
//        }
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//
//        frontLeft.setPower(frontLeftPower);
//        backLeft.setPower(backLeftPower);
//        frontRight.setPower(frontRightPower);
//        backRight.setPower(backRightPower);
//
//        // Update the arm state machine
//        // Telemetry for debugging and monitoring
//        telemetry.addData("Front Left Power", frontLeft.getPower());
//        telemetry.addData("Front Right Power", frontRight.getPower());
//        telemetry.addData("Back Left Power", backLeft.getPower());
//        telemetry.addData("Back Right Power", backRight.getPower());
//        telemetry.addData("left stick y pos", gamepad1.left_stick_y);
//        telemetry.addData("Drive forward is",driveForward);
//        telemetry.update();  // Update telemetry
//
//        // Control claw position with gamepad2.y button
//
//
//    }
//    private void initAprilTag() {
//
//        // Create the AprilTag processor the easy way.
//        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//
//        // Create the vision portal the easy way.
//        if (USE_WEBCAM) {
//            visionPortal = VisionPortal.easyCreateWithDefaults(
//                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
//        } else {
//            visionPortal = VisionPortal.easyCreateWithDefaults(
//                    BuiltinCameraDirection.BACK, aprilTag);
//        }
//
//    }
//
//    private void telemetryAprilTag() {
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//            if(detection.id==21){
//                motif = "gpp";
//            }
//            if(detection.id==22){
//                motif = "pgp";
//            } if(detection.id==23){
//                motif = "ppg";
//            }
//
//        }   // end for() loop
//
//        // Add "key" information to telemetry
//        telemetry.addLine(String.format("Motif is %s",motif));
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");
//
//    }   // end method telemetryAprilTag()
//}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.arcrobotics.ftclib.util.InterpLUT;

import java.util.List;


@TeleOp(name = "MecanumTeleop", group = "Concept")
public class MecanumTeleOpdeocde extends OpMode {
    decodeArm arm;
    double y;
    double x;
    double rx;
    boolean driveForward;
    // Motors for mecanum drive
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    boolean clawPos = false;
    boolean slidesExtend = false;  // Toggle flag for slide movement
    // Arm controller and state machine
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    boolean previousYState = false;
    boolean previousLeftBumperState = false;
    double clawRotationState = 0;
    public double pivotPos = 1;
    double clawOpen = 0.4;
    private Servo clawServo;

    private Servo pivotServo;
    public double slidesPos;
    public String motif;

    public boolean pivotMove = false;

    // Auto-align constants
    private static final double TURN_GAIN = 0.01;   // smaller = smoother/slower turn
    private static final double MAX_AUTO_TURN = 0.25; // limit auto turn power

// 0 backLeftMotor, 1 backRightMotor, 2 frontRightMotor, 3 frontLeftMotor

    @Override
    public void init() {
        driveForward = true;
        // Initialize mecanum drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        // Initialize the arm controller and state machine

    }

    @Override
    public void loop() {
        // Update AprilTag telemetry (still prints info)
        telemetryAprilTag();

        // Save CPU resources; can resume streaming when needed.
        if (gamepad1.dpad_down) {
            visionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
            visionPortal.resumeStreaming();
        }

        // Mecanum drive control (normal)
        if(driveForward) {
            y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            rx = gamepad1.right_stick_x;
        } else {
            y = gamepad1.left_stick_y * 1.1;
            x = -gamepad1.left_stick_x * 1.1;
            rx = -gamepad1.right_stick_x;
        }
        if(gamepad1.dpad_down){
            driveForward = !driveForward;
        }

        // --- AUTO-ALIGN: if left bumper held and tag detected, override rx to turn toward tag ---
        if (gamepad1.left_bumper) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            AprilTagDetection desired = null;
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    // pick the first known tag (you can filter IDs here if desired)
                    desired = detection;
                    break;
                }
            }
            if (desired != null) {
                // use bearing (degrees) to compute turn power
                double headingError = desired.ftcPose.bearing; // degrees
                double turnPower = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                // depending on sign convention, you may need to negate. If the robot turns the wrong way, flip sign.
                rx = turnPower;
                telemetry.addData("AutoAlign", "Turning to tag ID %d (bearing=%.2f) -> rx=%.3f", desired.id, headingError, rx);
            } else {
                telemetry.addData("AutoAlign", "Left bumper held but no valid tag seen");
            }
        }

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        // Update the arm state machine
        // Telemetry for debugging and monitoring
        telemetry.addData("Front Left Power", frontLeft.getPower());
        telemetry.addData("Front Right Power", frontRight.getPower());
        telemetry.addData("Back Left Power", backLeft.getPower());
        telemetry.addData("Back Right Power", backRight.getPower());
        telemetry.addData("left stick y pos", gamepad1.left_stick_y);
        telemetry.addData("Drive forward is",driveForward);
        telemetry.update();  // Update telemetry

        // Control claw position with gamepad2.y button

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
            if(detection.id==21){
                motif = "gpp";
            }
            if(detection.id==22){
                motif = "pgp";
            } if(detection.id==23){
                motif = "ppg";
            }

        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine(String.format("Motif is %s",motif));
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}
