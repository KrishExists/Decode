package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "MecanumTeleopMain_Switch_AutoAlign", group = "Main")
public class MainTele extends LinearOpMode {

    // ====== Hardware ======
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private DcMotor intake;
    private DcMotorEx outtake;
    private Servo linkage;

    PIDController rpmPID;


    // ====== Camera ======
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private double kP = 0.03;  // Proportional constant for alignment

    // ====== Timer ======
    private final ElapsedTime timer = new ElapsedTime();

    private enum IntakeState {
        INTAKE,
        OUTTAKE,
        RUNSLOW,
        OUTTAKE1, REST
    }

    private IntakeState currentState = IntakeState.REST;
    private IntakeState previousState = IntakeState.REST;

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

        rpmPID = new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        rpmPID.setTolerance(10);


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        linkage.setPosition(0.0);

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
            else if(gamepad2.a) {
                currentState = IntakeState.OUTTAKE1;
            }

            else {
                currentState = IntakeState.REST;
            }

            if (currentState != previousState) {
                timer.reset();
                previousState = currentState;
            }

            switch (currentState) {
                case INTAKE:
                    intake.setPower(0.8);
                    outtake.setPower(-0.2);
                    linkage.setPosition(0.8);
                    break;

                case OUTTAKE1:
                    if(timer.milliseconds() < 2800) {
                        outtake.setVelocity(2400);
                        intake.setPower(0.0);
                        linkage.setPosition(0.8);
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

                case OUTTAKE:
                    if (timer.milliseconds() < 700) {
                        outtake.setPower(-0.8);
                        intake.setPower(-0.5);
                        linkage.setPosition(0.8);
                    } else if (timer.milliseconds() < 3500) {
                        outtake.setPower(1.0);
                        intake.setPower(0.0);
                        linkage.setPosition(0.8);
                    } else if (timer.milliseconds() < 4000) {
                        outtake.setVelocity(2400);
                        linkage.setPosition(0.25);
                        intake.setPower(0.0);
                    } else {
                        outtake.setVelocity(2400);
                        linkage.setPosition(0.25);
                        intake.setPower(0.8);
                    }
                    break;

                case RUNSLOW:
                    intake.setPower(-1.0);
                    outtake.setPower(-0.8);
                    linkage.setPosition(0.8);
                    break;

                default:
                    intake.setPower(0.0);
                    outtake.setPower(0.0);
                    linkage.setPosition(0.8);
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
            telemetry.update();
        }
    }

    public boolean atRPM(){
        return rpmPID.atSetPoint();
    }

/*    public void setPIDPower(double targetRPM){
        double topVelocity = Math.abs(outtake.getVelocity());

        double currentRPM = (ticksPerSecToRPM(topVelocity)) / 2;
        telemetry.addData("Shooter Current RPM", currentRPM);
        rpmPID.setPID(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        double power = rpmPID.calculate(currentRPM, targetRPM);
        power += (targetRPM > 0) ? (ShooterConstants.kf * (targetRPM / ShooterConstants.MAX_RPM)) : 0.0;
        power = Range.clip(power, 0, 1);
        telemetry.addData("Power", power);
        outtake.setPower(power);
    }
*/


    public double ticksPerSecToRPM(double tps){
        return tps * 60.0 / ShooterConstants.TICKS_PER_REV;
    }

    public void periodic() {
        rpmPID.setPID(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
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
}
