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

    private Servo blocker;
    private DcMotorEx shooter, shooter2;
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

    // === Dashboard Adjustable Settings ===
    public static double linkagePos = 0.0;

    public static double blockerPos = 0.0;
    public static double shooterPower = 0.0;   // <-- NOW interpreted as TARGET RPM
    public static double intakePower = 0.0;

    @Override
    public void runOpMode() {

        linkage = hardwareMap.get(Servo.class, "Linkage");
        blocker = hardwareMap.get(Servo.class, "Blocker");

        shooter = hardwareMap.get(DcMotorEx.class, "Outtake");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Outtake2");

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "Intake");

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();

        initAprilTag();

        telemetry.addLine("Init Complete");
        telemetry.update();
        waitForStart();

        lastTime = getRuntime();

        while (opModeIsActive()) {

            // === SERVO ===
            linkage.setPosition(linkagePos);
            blocker.setPosition(blockerPos);

            // === SHOOTER RPM CONTROL ===
            spinToRpm(shooterPower);   // <-- shooterPower is TARGET RPM

            // === INTAKE ===
            intake.setPower(intakePower);

            // === APRILTAG ALIGN ===
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            if (detections != null && detections.size() > 0) {
                AprilTagDetection tag = detections.get(0);
                double error = tag.ftcPose.x;

                double currentTime = getRuntime();
                double dt = currentTime - lastTime;

                integralSum += error * dt;
                double derivative = (error - lastError) / dt;

                double output = (kP * error) + (kI * integralSum) + (kD * derivative);
                output = Range.clip(output, -0.4, 0.4);

                leftFront.setPower(output);
                leftRear.setPower(output);
                rightFront.setPower(-output);
                rightRear.setPower(-output);

                lastTime = currentTime;
                lastError = error;

                telemetry.addData("ATag ID", tag.id);
                telemetry.addData("Error", error);
            } else {
                stopDrive();
                telemetry.addLine("No Tags");
            }

            telemetry.addData("Target RPM", shooterPower);
            telemetry.addData("Shooter RPM", currentRPM());
            telemetry.addData("Shooter Power Output", shooter.getPower());
            telemetry.update();
        }

        visionPortal.close();
    }

    // ════════════════════════════════
    // SHOOTER RPM CONTROL METHODS
    // ════════════════════════════════

    public double currentRPM() {
        // Your team uses *velocity * 2.2* in the main code
        return shooter.getVelocity() * 2.2;
    }

    public void spinToRpm(double targetRPM) {
        double currRPM = currentRPM();
        double error = targetRPM - currRPM;

        double currentTime = getRuntime();
        double dt = currentTime - lastTime;

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);
        output = Range.clip(output, 0, 1);

        shooter.setPower(output);
        shooter2.setPower(output);

        lastError = error;
        lastTime = currentTime;
    }

    // ════════════════════════════════
    // Vision + Drive
    // ════════════════════════════════

    private void stopDrive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    private void initAprilTag() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        while (opModeInInit() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", visionPortal.getCameraState());
            telemetry.update();
            sleep(20);
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

        if (exposureControl != null) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(15, TimeUnit.MILLISECONDS);
        }
        if (gainControl != null) {
            gainControl.setGain(25);
        }
    }
}
