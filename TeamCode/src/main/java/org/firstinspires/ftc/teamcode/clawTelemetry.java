package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name = "Static Servo Telemetry + Auto Align (VisionPortal)", group = "Telemetry")
public class clawTelemetry extends LinearOpMode {

    // === Hardware ===
    private Servo linkage;
    private DcMotorEx shooter;

    private DcMotorEx shooter2;
    private DcMotor intake;
    private DcMotorEx leftFront, rightFront, leftRear, rightRear;

    // === Dashboard ===
    FtcDashboard dashboard;

    // === Vision ===
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // === PID Coefficients (Tunable in Dashboard) ===
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    // === Telemetry test variables ===
    public static double linkagePos = 0.0;
    public static double shooterPower = 0.0;
    public static double intakePower = 0.0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        linkage = hardwareMap.get(Servo.class, "Linkage");
        shooter = hardwareMap.get(DcMotorEx.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        shooter2 = hardwareMap.get(DcMotorEx.class, "Outtake2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();

        // Initialize VisionPortal and AprilTag
        initAprilTag();

        telemetry.addLine("Initialization Complete — Ready to Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            linkage.setPosition(linkagePos);
            shooter.setPower(shooterPower);
            shooter2.setPower(shooterPower);
            intake.setPower(intakePower);

            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            if (detections != null && detections.size() > 0) {
                AprilTagDetection tag = detections.get(0);
                double error = tag.ftcPose.x; // lateral offset (inches or meters)
                double currentTime = getRuntime();
                double dt = currentTime - lastTime;

                integralSum += error * dt;
                double derivative = (error - lastError) / dt;
                double output = (kP * error) + (kI * integralSum) + (kD * derivative);

                // Limit the power
                output = Range.clip(output, -0.4, 0.4);

                leftFront.setPower(output);
                leftRear.setPower(output);
                rightFront.setPower(-output);
                rightRear.setPower(-output);

                lastError = error;
                lastTime = currentTime;

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("X Offset (error)", error);
                telemetry.addData("PID Output", output);
            } else {
                stopDrive();
                telemetry.addLine("No AprilTags detected");
            }

            telemetry.addData("Linkage Pos", linkage.getPosition());
            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("Intake Power", intakePower);
            telemetry.update();
        }

        visionPortal.close();
    }

    private void initAprilTag() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("Opening camera...");
        telemetry.update();

        // Wait for the camera to start streaming before setting exposure/gain
        while (opModeInInit() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.update();
            sleep(20);
        }

        // Once streaming, THEN set exposure/gain
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

        if (exposureControl != null && gainControl != null) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(15, TimeUnit.MILLISECONDS);  // adjust for brightness
            gainControl.setGain(25); // max brightness, adjust if overexposed
            telemetry.addLine("Exposure and gain set!");
        } else {
            telemetry.addLine("Exposure/Gain controls not available!");
        }

        telemetry.update();
    }


    private void stopDrive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}
