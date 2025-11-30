package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.ShooterConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "NewTopic", group = "Main")
public class MainTeleNewLogic extends LinearOpMode {
    // Krish is Gay!!!:)
    // ====== Hardware ======
    private DcMotor frontLeftMotor;

    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    public static double kP = 0.01, kI = 0.0, kD = 0.0;
    private double integralSum = 0, lastError = 0, lastTime = 0;
    private DcMotor backRightMotor;

    private DcMotor intake;
    private boolean shootState;
    private DcMotorEx outtake;

    private DcMotorEx outtake2;
    private Servo linkage;

    PIDController rpmPID;
    private Double rpmThresh;


    // ====== Camera ======
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // ====== Timer ======
    private final ElapsedTime timer = new ElapsedTime();

//    private enum IntakeState {
//        INTAKE,
//        OUTTAKE,
//        SHOOT,
//        RUNSLOW,
//        OUTTAKE1, Outtake, REST
//    }

    private IntakeState currentState = IntakeState.REST;
    private IntakeState previousState = IntakeState.REST;
    boolean ball1Shot = false;
    boolean b2shot = false;
    boolean b3shot = false;
    boolean bwasPressed =false;
    public enum IntakeState { INTAKE, OUTTAKE, RUNSLOW, OUTTAKE1, OuttakeFar, OuttakeMid, REST }

    @Override
    public void runOpMode() throws InterruptedException {
        // ====== Initialize Motors ======
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        intake = hardwareMap.get(DcMotor.class, "Intake");
        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        linkage = hardwareMap.get(Servo.class, "Linkage");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake2 = hardwareMap.get(DcMotorEx.class, "Outtake2");
        rpmThresh = 100.0;


        rpmPID = new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        rpmPID.setTolerance(10);

        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);        //outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        outtake2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shootState = false;
        linkage.setPosition(0.0);
        boolean firstShotHappend = false;

        // ====== Initialize Camera ======
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Initialized — Waiting for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // ====== Intake Logic ======
            if (gamepad2.right_trigger > 0.2) {
                currentState = IntakeState.INTAKE;
            } else if (gamepad2.left_trigger > 0.2) {
                currentState = IntakeState.OUTTAKE;
            } else if (gamepad2.right_bumper) {
                currentState = IntakeState.RUNSLOW;
            }

            else if(gamepad2.y) {
                currentState = IntakeState.OuttakeMid;
            }

            else if(gamepad2.x) {
                currentState = IntakeState.OuttakeFar;
            }

            else if(gamepad2.a) {
                currentState = IntakeState.OUTTAKE1;
            }

            else{
                currentState = IntakeState.REST;
            }

            if (currentState != previousState) {
                timer.reset();
                previousState = currentState;
            }

            switch (currentState) {
                case INTAKE:
                    intake.setPower(0.8);
                    outtake.setPower(-0.4);
                    outtake2.setPower(-0.4);
                    linkage.setPosition(0.92);
                    break;

                case OUTTAKE1:
                    if(timer.milliseconds() < 1500) {
                        outtake.setVelocity(2000);
                        intake.setPower(0.0);
                        linkage.setPosition(0.92);
                    }

                    else if(timer.milliseconds() < 3300) {
                        outtake.setVelocity(2400);
                        linkage.setPosition(0.25);
                        intake.setPower(0.0);
                    }

                    else {
                        outtake.setVelocity(2400);
                        linkage.setPosition(0.25);
                        intake.setPower(0.8);
                    }
                    break;
//                case OUTTAKE:
//                    if (timer.milliseconds() < 700) {
//                        firstShotHappend = false;
//                        outtake.setPower(-0.8);
//                        intake.setPower(-0.5);
//                        linkage.setPosition(0.92);
//                    }else{
//                        outtake.setVelocity(2200);
//                        if(outtake.getVelocity()>=2200){
//                            // telemetry.addData("HI","HI");
//                            telemetry.update();
//                            firstShotHappend = true;
//                            linkage.setPosition(0.25);
//                            intake.setPower(0.8);
//                        }else{
//                            if(firstShotHappend){
//                                linkage.setPosition(0.25);
//                            }else{
//                                linkage.setPosition(0.92);
//                            }
//                            intake.setPower(0.0);
//                        }
//                    }
//
//                    break;
                case OuttakeMid:
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
                    if(timer.milliseconds()<500){
                        linkage.setPosition(0.47);
                    }else {
                        spinToRpm(6000);
                        if (currentRPM() > 3700&&currentRPM()<6000) {
                            intake.setPower(1);
                        } else {
                            intake.setPower(0);
                        }
                    }
                    break;

                case OUTTAKE:
                    if(timer.milliseconds()<500){
                        linkage.setPosition(0.25);
                    }else {
                        spinToRpm(1500);
                        if (currentRPM() > 1300&&currentRPM()<1700) {
                            intake.setPower(1);
                        } else {
                            intake.setPower(0);
                        }
                    }
                    break;
                case RUNSLOW:
                    linkage.setPosition(0.92);
                    if(timer.milliseconds()<500){
                        intake.setPower(-1.0);
                        outtake.setPower(-0.8);
                        outtake2.setPower(-0.8);
                    }else {
                        intake.setPower(0);
                        outtake.setPower(-0.8);
                        outtake2.setPower(-0.8);
                    }
                    break;

                default:
                    timer.reset();
                    intake.setPower(0.0);
                    outtake.setPower(0.0);
                    outtake2.setPower(0.0);
                    linkage.setPosition(0.92);

                    break;
            }

            // ====== Drive Control ======
            if (gamepad2.left_bumper) {
                autoAlignToTag();
            } else {
                manualDrive();
            }

            telemetry.addData("State", currentState);
            telemetry.addData("Timer (ms)", timer.milliseconds());
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Outtake Power", outtake.getPower());
            telemetry.addData("Linkage Pos", linkage.getPosition());
            telemetry.addData("Outtake Shooter: ", outtake.getVelocity());
            telemetry.addData("Outtake rpm",currentRPM());
            telemetry.addData("Ball3 shot",b3shot);
            telemetry.update();
        }
    }


    // ====== Helper Methods ======

    private void manualDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private void autoAlignToTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) {
            telemetry.addLine("No Tag Detected");
            stopDrive();
            return;
        }

        // Just pick the first visible tag
        AprilTagDetection tag = detections.get(0);
        double error = tag.ftcPose.x; // x-offset from center
        double turnPower = error * kP;

        frontLeftMotor.setPower(turnPower);
        backLeftMotor.setPower(turnPower);
        frontRightMotor.setPower(-turnPower);
        backRightMotor.setPower(-turnPower);

        telemetry.addData("Tag ID", tag.id);
        telemetry.addData("Error", error);
        telemetry.addData("Turn Power", turnPower);
    }

    private void stopDrive() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public boolean upToRpm(double rpm) {
        double curr = currentRPM();
        if(curr>rpm-rpmThresh){
            return true;
        }else{
            return false;
        }
    }

    public double currentRPM() {
        return outtake.getVelocity() * 2.2;
    }

    public void resetBooleans(){
        ball1Shot = false;
        b2shot = false;
        b3shot = false;
    }

    // -- linkage --
    public void setLinkage(double pos) {
        linkage.setPosition(pos);
    }

    public double getVelocity() { return outtake.getVelocity();}

    public void setPower(double power){
        outtake.setPower(power);
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
}
