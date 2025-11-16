package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class DecodeController {

    // Hardware
    private DcMotorEx shooter;
    private DcMotorEx intake;
    private Servo linkage;
    private double power;

    // Vision
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private InterpLUT lut;
    private HardwareMap hardwareMap;

    public States state;

    // Button edge detection
    private boolean wasBButtonPressed = false;

    public enum States {
        Intake,
        Rest,
        Outake,
        Calculate
    }

    // Constructor
    public DecodeController(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initHardware(hardwareMap);
        initAprilTag();
    }

    private void initHardware(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        shooter = hardwareMap.get(DcMotorEx.class, "Outtake");
        linkage = hardwareMap.get(Servo.class, "Linkage");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        state = States.Rest;
    }

    public void controllerInit() {
        state = States.Intake;
    }

    public StateMachine shooterMachine(Gamepad gamepad1, Gamepad gamepad2) {
        return new StateMachineBuilder()

                // ====== Intake ======
                .state(States.Intake)
                .onEnter(() -> {
                    state = States.Intake;
                    if (visionPortal != null) visionPortal.resumeStreaming();
                })
                .loop(() -> {
                    intake.setPower(0.8);
                    shooter.setPower(0.0);
                    linkage.setPosition(0.0);
                })
                .transition(() -> isBPressed(gamepad2), States.Calculate)

                // ====== Calculate ======
                .state(States.Calculate)
                .loop(() -> {
                    state = States.Calculate;
                    if (aprilTag != null) {
                        List<AprilTagDetection> detections = aprilTag.getDetections();

                        if (detections != null && !detections.isEmpty()) {
                            for (AprilTagDetection detection : detections) {
                                if (detection.id <= 23) continue;
                                double range = detection.ftcPose.range;
                                power = (lut != null) ? lut.get(range) : 1.0;
                            }
                        } else {
                            power = 1.0;
                        }
                    }
                })
                .onExit(() -> {
                    if (visionPortal != null) visionPortal.stopStreaming();
                })
                .transition(() -> isBPressed(gamepad2), States.Outake)

                // ====== Outake ======
                .state(States.Outake)
                .onEnter(() -> {
                    state = States.Outake;
                    intake.setPower(-0.25);
                    shooter.setPower(-0.1);
                })
                .loop(() -> {
                    linkage.setPosition(0.2);
                    intake.setPower(0.25);
                    shooter.setPower(power);
                })
                .transition(() -> isBPressed(gamepad2), States.Rest)

                // ====== Rest ======
                .state(States.Rest)
                .loop(() -> {
                    state = States.Rest;
                    intake.setPower(0);
                    shooter.setPower(0);
                    linkage.setPosition(0.0);
                })
                .transition(() -> isBPressed(gamepad2), States.Intake)

                .build();
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    // === Utility: falling edge detection (like ArmController) ===
    private boolean isBPressed(Gamepad gamepad2) {
        boolean isPressed = gamepad2.b;
        boolean fallingEdge = !wasBButtonPressed && isPressed;
        wasBButtonPressed = isPressed;
        return fallingEdge;
    }

    /*
    private void setupLUT() {
        lut = new InterpLUT();
        lut.add(2.7, 0.5);
        lut.add(3.6, 0.75);
        lut.add(4.1, 0.9);
        lut.add(5.0, 1.0);
        lut.createLUT();
    }
    */
}
