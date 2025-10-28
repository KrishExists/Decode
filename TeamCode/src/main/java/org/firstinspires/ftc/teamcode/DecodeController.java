package org.firstinspires.ftc.teamcode;

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

    private DcMotorEx shooter;
    private DcMotorEx intake;
    private Servo linkage;
    private double power;

    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private InterpLUT lut;

    private HardwareMap hardwareMap; // ✅ store reference for later use
    public States state;

    public enum States {
        Intake,
        Rest,
        Outake,
        Calculate
    }

    // Constructor
    public DecodeController(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap; // ✅ fix: save it for later
        init(hardwareMap);
        initAprilTag();
        // setupLUT(); // optional lookup table
    }

    private void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        shooter = hardwareMap.get(DcMotorEx.class, "Outtake");
        linkage = hardwareMap.get(Servo.class, "Linkage");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        state = States.Rest; // ✅ ensure default state
    }

    public void controllerInit() {
        state = States.Intake;
    }

    public StateMachine shooterMachine(Gamepad gamepad1, Gamepad gamepad2) {
        return new StateMachineBuilder()

                // ====== Intake ======
                .state(States.Intake)
                .onEnter(() -> visionPortal.resumeStreaming())
                .loop(() -> {
                    intake.setPower(0.5);
                    shooter.setPower(0.0);
                    linkage.setPosition(1.0);
                })
                .transition(() -> gamepad2.b, States.Calculate)

                // ====== Calculate ======
                .state(States.Calculate)
                .loop(() -> {
                    List<AprilTagDetection> detections = aprilTag.getDetections();

                    for (AprilTagDetection detection : detections) {
                        if (detection.id <= 23) continue; // filter if needed

                        double range = detection.ftcPose.range;
                        // power = lut != null ? lut.get(range) : 1.0;
                        power = 1.0; // default if LUT not used
                    }
                })
                .onExit(() -> visionPortal.stopStreaming())
                .transition(() -> gamepad2.b, States.Outake)

                // ====== Outake ======
                .state(States.Outake)
                .loop(() -> {
                    linkage.setPosition(0.3);
                    intake.setPower(0.25);
                    shooter.setPower(power);
                })
                .transition(() -> gamepad2.b, States.Rest)

                // ====== Rest ======
                .state(States.Rest)
                .loop(() -> {
                    intake.setPower(0);
                    shooter.setPower(0);
                })
                .transition(() -> gamepad2.b, States.Intake)

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
