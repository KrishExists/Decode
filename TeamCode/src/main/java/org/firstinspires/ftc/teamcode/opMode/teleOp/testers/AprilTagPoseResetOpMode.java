package org.firstinspires.ftc.teamcode.opMode.teleOp.testers;

import android.nfc.Tag;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "AprilTag Pose Reset Test")
public class AprilTagPoseResetOpMode extends LinearOpMode {

    private MecanumDrive drive;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // ---- CONSTANTS ----
    private static final Pose2d CAMERA_ON_ROBOT =
            new Pose2d(3.2, 0.58, 0.0); // 3.2 in FRONT, 0.58 LEFT
    private static final Pose2d TAG_FIELD_POSE =
            new Pose2d(-74, 72, Math.PI);

    private boolean lastButton = false;

    @Override
    public void runOpMode() {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );

        waitForStart();

        while (opModeIsActive()) {

            drive.updatePoseEstimate();

            // ---- Button press (edge-detected) ----
            boolean pressed = gamepad1.a;
            if (pressed && !lastButton) {
                AprilTagDetection detection = getBestTag();
                if (detection != null) {
                    resetPoseFromTag(detection);
                }
            }
            lastButton = pressed;

            // ---- Telemetry ----
            Pose2d pose = drive.localizer.getPose();
            telemetry.addData("X", pose.position.x);
            telemetry.addData("Y", pose.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.real));
            telemetry.update();
        }
    }

    // -------------------------
    // CORE POSE RESET LOGIC
    // -------------------------
    private void resetPoseFromTag(AprilTagDetection detection) {

        // FTC → Road Runner conversion
        Pose2d tagCameraPose = new Pose2d(
                detection.ftcPose.z,          // forward
                -detection.ftcPose.x,         // left
                Math.toRadians(detection.ftcPose.yaw)
        );

        // Convert Pose2d → Twist2d for proper SE(2) addition
        Twist2d tagTwist = tagCameraPose.inverse().log();
        Twist2d cameraTwist = CAMERA_ON_ROBOT.inverse().log();

        // Manually add the twist components
        Twist2d totalTwist = new Twist2d(
                tagTwist.line.plus(cameraTwist.line),
                tagTwist.angle + cameraTwist.angle
        );



        // Apply the total twist relative to TAG_FIELD_POSE
        Pose2d robotFieldPose = TAG_FIELD_POSE.exp(totalTwist);

        // Set the robot pose
        drive.localizer.setPose(robotFieldPose);
    }


    // -------------------------
    // Choose closest visible tag
    // -------------------------
    private AprilTagDetection getBestTag() {
        AprilTagDetection best = null;
        double bestDist = Double.MAX_VALUE;

        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d.ftcPose != null && d.ftcPose.range < bestDist) {
                best = d;
                bestDist = d.ftcPose.range;
            }
        }
        return best;
    }
}
