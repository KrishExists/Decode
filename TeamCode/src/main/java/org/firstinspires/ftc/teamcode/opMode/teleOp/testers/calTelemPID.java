package org.firstinspires.ftc.teamcode.opMode.teleOp.testers;

import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
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

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp(name = "Static Servo ", group = "Telemetry")
public class calTelemPID extends LinearOpMode {

    // === Hardware ===
    private Servo linkage;
    private Servo blocker;
    private DcMotorEx shooter, shooter2;
    private DcMotor intake;
    private DcMotorEx leftFront, rightFront, leftRear, rightRear;
    private DcMotorEx transfer;

    // === Dashboard ===
    FtcDashboard dashboard;

    // === Vision ===
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // === PID Coefficients (Tunable in Dashboard) ===
    public static double kP = 0.0005;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static double kS = 0.05;
    public static double kV = 0.0002;
    public static double kA = 0.0;

    private PIDFController shooterPIDF;
    private SimpleMotorFeedforward shooterFF;

    // === Dashboard Adjustable Settings ===
    public static double linkagePos = 0.0;
    public static double transferPower = 0.0;
    public static double blockerPos = 0.55;
    public static double shooterPower = 0.0;   // <-- NOW interpreted as TARGET RPM
    public static double intakePower = 0.0;
    public static double shooterPoser = 0.0;
    public static boolean usesecond = true;

    @Override
    public void runOpMode() {

        // === Hardware Init ===
        linkage = hardwareMap.get(Servo.class, "Linkage");
        blocker = hardwareMap.get(Servo.class, "blocker");
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");

        shooter = hardwareMap.get(DcMotorEx.class, "Outtake");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Outtake2");

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
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
        blocker.setPosition(0.55);
        telemetry.addLine("Init Complete");
        telemetry.update();

        // === Shooter PIDF Init ===
        shooterPIDF = new PIDFController(kP, kI, kD, kF);
        shooterPIDF.setTolerance(50); // RPM tolerance
        shooterFF = new SimpleMotorFeedforward(kS, kV, kA);

        waitForStart();

        while (opModeIsActive()) {
            shooterPIDF.setPIDF(kP, kI, kD, kF);
            shooterFF = new SimpleMotorFeedforward(kS, kV, kA);
            // === SERVO ===
            linkage.setPosition(linkagePos);
            blocker.setPosition(blockerPos);
            transfer.setPower(transferPower);

            // === SHOOTER RPM CONTROL ===
            if(shooterPoser==0){
                spinToRpm(shooterPower);
            }else{
                shooter.setPower(shooterPoser);
                if(usesecond){
                    shooter2.setPower(shooterPoser);

                }
            }

            // === INTAKE ===
            intake.setPower(intakePower);

            // === APRILTAG ALIGN ===
//            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
//
//            if (detections != null && detections.size() > 0) {
//                AprilTagDetection tag = detections.get(0);
//                double error = tag.ftcPose.x;
//
//                double currentTime = getRuntime();
//                double dt = currentTime - lastTime;
//
//                integralSum += error * dt;
//                double derivative = (error - lastError) / dt;
//
//                double output = (kP * error) + (kI * integralSum) + (kD * derivative);
//                output = Range.clip(output, -0.4, 0.4);
//
//                leftFront.setPower(output);
//                leftRear.setPower(output);
//                rightFront.setPower(-output);
//                rightRear.setPower(-output);
//
//                lastTime = currentTime;
//                lastError = error;
//
//                telemetry.addData("ATag ID", tag.id);
//                telemetry.addData("Error", error);
//            } else {
//                stopDrive();
//                telemetry.addLine("No Tags");
//            }

            // === TELEMETRY ===
            telemetry.addData("Target RPM", shooterPower);
            telemetry.addData("Shooter RPM", currentRPM());
            telemetry.addData("Shooter1 RPM", currentRPM1());
            telemetry.addData("Shooter PID Output", shooterPIDF.getP());
            telemetry.addData("Feedforward Power", shooterFF.calculate(shooterPower,0));
            telemetry.update();
        }

        visionPortal.close();
    }

    // ════════════════════════════════
    // SHOOTER RPM CONTROL METHODS
    // ════════════════════════════════

    public double currentRPM() {
        return shooter.getVelocity() * 2.2;
    }
    public double currentRPM1() {
        return shooter2.getVelocity() * 2.2;
    }

    public void spinToRpm(double targetRPM) {
        double currRPM = currentRPM();

        // PID feedback
        double pidPower = shooterPIDF.calculate(currRPM, targetRPM);

        // Feedforward
        double ffPower = shooterFF.calculate(targetRPM, 0);

        // Combine and clip
        double power = Range.clip(pidPower + ffPower, 0, 1);

        shooter.setPower(power);
        if(usesecond){
            shooter2.setPower(power);
        }
        telemetry.addData("PID Output", pidPower);
        telemetry.addData("Feedforward Output", ffPower);
        telemetry.addData("Combined Power", power);
    }

    // ════════════════════════════════
    // Drive Methods
    // ════════════════════════════════
    private void stopDrive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}
