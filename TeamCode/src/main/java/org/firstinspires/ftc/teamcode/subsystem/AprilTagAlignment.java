package org.firstinspires.ftc.teamcode.subsystem;

import android.telecom.TelecomManager;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.List;


@Config
public class AprilTagAlignment implements  Subsystem{
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    public static double kP = 0.05;
    public static int targetTagID = 20;

    public static double angleTolerance = 0.01;

    public static double maxPower = 0.5;

    public boolean alignmentActive = false;
    private boolean isAligned = false;

    Telemetry telemetry;

    public AprilTagAlignment(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;
        leftFront = map.get(DcMotor.class, "frontLeftMotor");
        leftBack = map.get(DcMotor.class, "backLeftMotor");
        rightBack = map.get(DcMotor.class, "backRightMotor");
        rightFront = map.get(DcMotor.class, "backRightMotor");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack
                .setDirection(DcMotor.Direction.REVERSE);
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                map.get(WebcamName.class, "Webcam 1"),
                aprilTagProcessor
        );
    }



    @Override
    public void init() {
        telemetry.addData("AprilTag Alignment", "initialized");
        telemetry.addData("Target Tag ID", targetTagID);
        telemetry.update();

    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        if (alignmentActive){
            performAlignment();
        }

        telemetry.addLine("AprilTag");
        telemetry.addData("Mode", alignmentActive ? "AUTO ALIGN" : "MANUAL");
        if(alignmentActive){
            if (isTagDetected()){
                telemetry.addData("Tag Detected", "ID %d", targetTagID);
                telemetry.addData("Aligned", isAligned ? "YES" : "NO");
            }
            else {
                telemetry.addLine("Target tag not visible");
            }
        }


        if (gamepad1.x){
            alignmentActive = true;
        } else {
            alignmentActive = false;
            isAligned = false;
            stopMotors();
        }

    }

    public boolean isAlignmentActive(){
        return alignmentActive;
    }

    public void performAlignment(){
        AprilTagDetection aprilTagDetection = getTagByID(targetTagID);

        if (aprilTagDetection == null){
            stopMotors();
            isAligned = false;
            return;
        }

        double x = aprilTagDetection.ftcPose.x;
        double y = aprilTagDetection.ftcPose.y;
        double angleToTag = Math.atan2(x,y);
        double error = Math.toDegrees(angleToTag);

        double rotationPower = kP * error;

        rotationPower =  Math.max(-maxPower, Math.min(maxPower, rotationPower));

        leftFront.setPower(rotationPower);
        leftBack.setPower(rotationPower);
        rightFront.setPower(rotationPower);
        rightBack.setPower(rotationPower);

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("power", rotationPower);
        telemetry.addData("error angle", (angleToTag));
        if (Math.abs(error) < angleTolerance){
            isAligned = true;
        } else {
            isAligned = false;
        }
    }

    private void stopMotors(){
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public double getCurrentAngle(){
        AprilTagDetection aprilTagDetection = getTagByID(targetTagID);
        return aprilTagDetection != null ? aprilTagDetection.ftcPose.yaw : 0.0;
    }

    public boolean isTagDetected(){
        return getTagByID(targetTagID) != null;
    }

    public boolean isAligned() {
        return isAligned;
    }

    private AprilTagDetection getTagByID (int id){
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections){
            if (detection.id == id){
                return detection;
            }
        }
        return null;
    }

    public void close(){
        visionPortal.close();
    }
}
