//package org.firstinspires.ftc.teamcode.subsystem;
//
//
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Hardware;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import java.util.List;
//
//
//public class Vision {
//    private final VisionPortal portal;
//    private final AprilTagProcessor aprilTag;
//    private final Hardware hw;
//    private double kP_align = 0.03;
//
//
//    public Vision(Hardware hardware) {
//        hw = hardware;
//        aprilTag = new AprilTagProcessor.Builder().build();
//        portal = new VisionPortal.Builder()
//                .setCamera(hw.webcam)
//                .addProcessor(aprilTag)
//                .build();
//    }
//
//
//    public List<AprilTagDetection> getDetections() {
//        return aprilTag.getDetections();
//    }
//
//
//    // Align helper returns a turn power (signed) or 0 if no tag
//    public double computeAlignPower() {
//        List<AprilTagDetection> det = getDetections();
//        if (det.isEmpty()) return 0.0;
//        AprilTagDetection tag = det.get(0);
//        double error = tag.ftcPose.x;
//        return error * kP_align;
//    }
//
//    public AprilTagDetection autoAlignToTag(Drivetrain drive) {
//        List<AprilTagDetection> detections = aprilTag.getDetections();
//        if (detections.isEmpty()) {
//            drive.stop();
//            return null;
//        }
//
//
//// Use closest detection
//        AprilTagDetection tag = detections.get(0);
//        return tag;
//
//
//
//
//    }
//}