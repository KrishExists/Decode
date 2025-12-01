package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "MecanumTeleopMain_Switch_AutoAlign", group = "Main")
public class MainTele extends LinearOpMode {

    // ====== Hardware ======
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor intake;
    private DcMotorEx outtake, outtake2;
    private Servo linkage;

    private Servo blocker;
    // ====== Shooter PID ======
    public static double kP = 0.01, kI = 0.0, kD = 0.0;
    private double integralSum = 0, lastError = 0, lastTime = 0;

    // ====== Vision ======
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private double kP_align = 0.03;

    private final ElapsedTime timer = new ElapsedTime();

    public enum IntakeState { INTAKE, OUTTAKE, RUNSLOW, OUTTAKE1, OuttakeFar, OuttakeMid, Transfer, REST }
    private IntakeState currentState = IntakeState.REST;
    private IntakeState previousState = IntakeState.REST;



    @Override
    public void runOpMode() throws InterruptedException {

        // ====== Initialize Hardware ======
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        blocker = hardwareMap.get(Servo.class, "Blocker");

        intake = hardwareMap.get(DcMotor.class, "Intake");
        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        outtake2 = hardwareMap.get(DcMotorEx.class, "Outtake2");
        linkage = hardwareMap.get(Servo.class, "Linkage");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Match working PID setup
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        linkage.setPosition(0.92);

        // ====== Vision Initialization ======
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        lastTime = getRuntime();

        telemetry.addLine("Initialized — Waiting for start");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        lastTime = getRuntime();
        lastError = 0;
        integralSum = 0;

        while (opModeIsActive()) {

            // ====== Intake State Machine ======
            if (gamepad2.right_trigger > 0.2) currentState = IntakeState.INTAKE;
            else if(gamepad2.dpad_left) currentState = IntakeState.Transfer;
            else if (gamepad2.left_trigger > 0.2) currentState = IntakeState.OUTTAKE;
            else if (gamepad2.right_bumper) currentState = IntakeState.RUNSLOW;
            else if (gamepad2.a) currentState = IntakeState.OUTTAKE1;
            else if (gamepad2.x) currentState = IntakeState.OuttakeFar;
            else if (gamepad2.y) currentState = IntakeState.OuttakeMid;
            else currentState = IntakeState.REST;

            if (currentState != previousState) {
                timer.reset();
                previousState = currentState;
            }

            switch (currentState) {

                case INTAKE:
                    intake.setPower(0.8);
                    outtake.setPower(-0.2);
                    outtake2.setPower(-0.2);
                    linkage.setPosition(0.92);
                   // blocker.setPosition(0);

                    break;

                case Transfer:
                   // blocker.setPosition(0.6);
                    intake.setPower(-0.5);
                    outtake.setPower(-0.2);
                    outtake2.setPower(-0.2);


                case OUTTAKE1:
                  //  blocker.setPosition(0);
                    if (timer.milliseconds() < 1500) {
                        outtake.setVelocity(2000);
                        intake.setPower(0);
                        linkage.setPosition(0.92);
                    } else if (timer.milliseconds() < 3300) {
                        outtake.setVelocity(2400);
                        intake.setPower(0);
                        linkage.setPosition(0.25);
                    } else {
                        outtake.setVelocity(2400);
                        intake.setPower(0.8);
                        linkage.setPosition(0.25);
                    }
                    break;

                case OUTTAKE:
                  //  blocker.setPosition(0.6);
                    if (timer.milliseconds() < 700) {
                        outtake.setPower(-0.8);
                        outtake2.setPower(-0.8);
                        intake.setPower(-0.5);
                        linkage.setPosition(0.92);
                    } else if (timer.milliseconds() < 2500) {
                        spinToRpm(2700); // working target RPM
                        intake.setPower(0);
                        linkage.setPosition(0.92);
                    } else if (timer.milliseconds() < 3000) {
                        spinToRpm(2700);
                        intake.setPower(0);
                        linkage.setPosition(0.25);
                    } else if (timer.milliseconds() < 4000) {
                        spinToRpm(2700);
                        intake.setPower(0.8);
                        linkage.setPosition(0.25);
                    } else {
                        spinToRpm(2700);
                        intake.setPower(0.8);
                    }
                    break;

                case OuttakeMid:
                  //  blocker.setPosition(0.6);
                    if(timer.milliseconds()<500){
                        linkage.setPosition(0.47);
                    }else {
                        spinToRpm(3500);
                        if (currentRPM() > 3400&&currentRPM()<3600) {
                            intake.setPower(1);
                        } else {
                            intake.setPower(0);
                        }
                    }
                    break;

                case OuttakeFar:
                 //   blocker.setPosition(0.6);
                    if(timer.milliseconds()<500){
                        linkage.setPosition(0.47);
                    }else {
                        spinToRpm(6000);
                        if (currentRPM() > 4000&&currentRPM()<6000) {
                            intake.setPower(1);
                        } else {
                            intake.setPower(0);
                        }
                    }
                    break;


                case RUNSLOW:
                    intake.setPower(-1.0);
                    outtake.setPower(-0.8);
                    outtake2.setPower(-0.8);
                    linkage.setPosition(0.92);
                    break;

                default:
                    intake.setPower(0.0);
                   // blocker.setPosition(0.6);
                    outtake.setPower(0.0);
                    outtake2.setPower(0.0);
                    linkage.setPosition(0.47);
                    break;
            }

            // ====== Drive ======
            if (gamepad2.left_bumper) autoAlignToTag();
            else manualDrive();

            telemetry.addData("State", currentState);
            telemetry.addData("Shooter RPM", currentRPM());
            telemetry.addData("Outtake Power", outtake.getPower());
            telemetry.update();
        }
    }

    // ====== Shooter Methods ======
    public double currentRPM() {
        return outtake.getVelocity() * 2.2; // same as clawTelemetry
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

        outtake.setPower(output);
        outtake2.setPower(output);

        lastError = error;
        lastTime = currentTime;
    }

    // ====== Drive ======
    private void manualDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        frontLeftMotor.setPower((y + x + rx) / denominator);
        backLeftMotor.setPower((y - x + rx) / denominator);
        frontRightMotor.setPower((y - x - rx) / denominator);
        backRightMotor.setPower((y + x - rx) / denominator);
    }

    private void autoAlignToTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) {
            stopDrive();
            telemetry.addLine("No Tag");
            return;
        }

        AprilTagDetection tag = detections.get(0);
        double error = tag.ftcPose.x;

        double turnPower = error * kP_align;

        frontLeftMotor.setPower(turnPower);
        backLeftMotor.setPower(turnPower);
        frontRightMotor.setPower(-turnPower);
        backRightMotor.setPower(-turnPower);
    }

    private void stopDrive() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
