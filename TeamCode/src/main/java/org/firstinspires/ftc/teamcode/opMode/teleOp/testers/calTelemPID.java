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
@TeleOp(name = "Static Servo ", group = "Testers")
public class calTelemPID extends LinearOpMode {

    // === Hardware ===
    private Servo linkage;
    private Servo blocker;
    private Servo turret1;
    private Servo turret2;
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
    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.000248;

    private PIDFController shooterPIDF;

    // === Dashboard Adjustable Settings ===
    public static double linkagePos = 0.0;
    public static double transferPower = 0.0;
    public static double blockerPos = 0.55;
    public static double turretpos = 0.5;
    public static double turret1pos = 0.5;

    public static double shooterPower = 0.0;   // <-- NOW interpreted as TARGET RPM
    public static double intakePower = 0.0;
    public static double shooterPoser = 0.0;
    public static boolean usesecond = true;
    public static boolean usefirst = true;

    @Override
    public void runOpMode() {

        // === Hardware Init ===
        linkage = hardwareMap.get(Servo.class, "Linkage");
        turret1 = hardwareMap.get(Servo.class, "TurretServo");
        turret2 = hardwareMap.get(Servo.class, "TurretServo2");

        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
        blocker = hardwareMap.get(Servo.class, "blocker");
        shooter = hardwareMap.get(DcMotorEx.class, "Outtake");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Outtake2");

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

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
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();
        blocker.setPosition(0.55);
        telemetry.addLine("Init Complete");
        telemetry.update();

        // === Shooter PIDF Init ===
        shooterPIDF = new PIDFController(kP, kI, kD, kF);

        shooterPIDF.setTolerance(50); // RPM tolerance

        waitForStart();

        while (opModeIsActive()) {
            shooterPIDF.setPIDF(kP, kI, kD, kF);
            // === SERVO ===
            linkage.setPosition(linkagePos);
            turret1.setPosition(turretpos);
            turret2.setPosition(turretpos);
            transfer.setPower(transferPower);

            // === SHOOTER RPM CONTROL ===
            if(shooterPoser==0){
                spinToRpm(shooterPower);
            }else{
                if(usefirst){
                    shooter.setPower(shooterPoser);

                }
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
            telemetry.update();
        }

        visionPortal.close();
    }

    // ════════════════════════════════
    // SHOOTER RPM CONTROL METHODS
    // ════════════════════════════════

    public double currentRPM() {
        return Math.abs(shooter.getVelocity() * 2.14);
    }
    public double currentRPM1() {
        return Math.abs(shooter2.getVelocity() * 2.14);
    }

    public void spinToRpm(double targetRPM) {
        double currRPM = currentRPM();

        // PID feedback
        double pidPower = shooterPIDF.calculate(currRPM, targetRPM);

        // Feedforward

        // Combine and clip
        double power = Range.clip(pidPower , 0, 1);

        double currRPM2 = currentRPM1();

        // PID feedback
        double pidPower2 = shooterPIDF.calculate(currRPM2, targetRPM);

        // Feedforward

        // Combine and clip
        double power2 = Range.clip(pidPower2, 0, 1);
        if(usefirst){
            shooter.setPower(power);
        }
        if(usesecond){
            shooter2.setPower(power2);
        }
        telemetry.addData("Combined Power", power);
        telemetry.addData("Combined Power2", power2);

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
