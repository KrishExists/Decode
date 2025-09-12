//package org.firstinspires.ftc.teamcode;
//
//import static java.lang.Thread.sleep;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.sfdev.assembly.state.StateMachine;
//import com.sfdev.assembly.state.StateMachineBuilder;
//
//public class decodeArm {
//    private DcMotorEx spinIntake;
//    private DcMotorEx rightShooter;
//    private DcMotorEx leftShooter;
//
//    // Servos
//    private Servo armLeft;
//    private Servo armRight;
//    private Servo idle1;
//    private Servo idle2;
//    private Servo idle3;
//    private Servo idle4;
//    String apriltagneeded;
//    int totalShot = 0;
//    private ArmState currentState;
//
//    public enum ArmState {
//        Hover,
//        Intake,
//        Loaded,
//        Shoot1,
//    }
//
//    public decodeArm(HardwareMap hardwareMap, String aprilTag) {
//        apriltagneeded = aprilTag;
//        leftShooter = hardwareMap.get(DcMotorEx.class, "leftOutake");
//        rightShooter = hardwareMap.get(DcMotorEx.class, "rightOutake");
//        leftShooter.setDirection(DcMotorEx.Direction.FORWARD);
//        rightShooter.setDirection(DcMotorEx.Direction.REVERSE);
//        leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        spinIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        leftShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        spinIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        // Run without encoders
//        leftShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        rightShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        spinIntake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        // Initialize state
//        currentState = ArmState.Hover;
//    }
//    private boolean wasBButtonPressed = false;
//    public StateMachine machine(Gamepad gamepad1){
//        return new StateMachineBuilder()
//                .state(ArmState.Hover)
//                .loop(()->{
//                    currentState = ArmState.Hover;
//                    //shooters
//                    leftShooter.setPower(0);
//                    rightShooter.setPower(0);
//                    //intakes
//                    spinIntake.setPower(0);
//                })
//                .transition(()->gamepad1.b,ArmState.Intake)
//                .transition(()->gamepad1.right_bumper,ArmState.Shoot1)
//                .state(ArmState.Intake)
//                .loop(()->{
//                    currentState = ArmState.Intake;
//                    //shooters
//                    leftShooter.setPower(0);
//                    rightShooter.setPower(0);
//                    //intakes
//                    spinIntake.setPower(1);
//                }).transition(()-> gamepad1.right_bumper,ArmState.Hover)
//                .state(ArmState.Shoot1)
//                .loop(() ->{
//                    currentState = ArmState.Shoot1;
//                    leftShooter.setPower(1);
//                    rightShooter.setPower(1);
//                    spinIntake.setPower(0);
//                }
//
//
//    }
//
//}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import com.arcrobotics.ftclib.util.InterpLUT;

public class decodeArm {
    private DcMotorEx spinIntake;
    private DcMotorEx rightShooter;
    private DcMotorEx leftShooter;

    // Servos
    private Servo armLeft;
    private Servo armRight;
    private Servo idle1;
    private Servo idle2;
    private Servo idle3;
    private Servo idle4;
    private String apriltagneeded;
    private int totalShot = 0;
    private ArmState currentState;

    // Gamepad button tracking
    private boolean prevB = false;
    private boolean prevRB = false;

    public enum ArmState {
        Hover,
        Intake,
        Loaded,
        Shoot1,
        Shoot2,
        Shoot3
    }

    public decodeArm(HardwareMap hardwareMap, String aprilTag) {
        apriltagneeded = aprilTag;
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftOutake");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightOutake");
        spinIntake = hardwareMap.get(DcMotorEx.class, "intake"); // Make sure this is initialized!
        idle1 = hardwareMap.get(Servo.class, "idle1");
        idle2 = hardwareMap.get(Servo.class, "idle2");
        idle3 = hardwareMap.get(Servo.class, "idle3");
        idle4 = hardwareMap.get(Servo.class, "idle4");

        leftShooter.setDirection(DcMotorEx.Direction.FORWARD);
        rightShooter.setDirection(DcMotorEx.Direction.REVERSE);

        leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spinIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spinIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        spinIntake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        currentState = ArmState.Hover;
    }

    // Edge detection helpers
    private  boolean risingEdge(boolean current, boolean previous) {
        return current && !previous;
    }

    private  boolean fallingEdge(boolean current, boolean previous) {
        return !current && previous;
    }

    public StateMachine machine(Gamepad gamepad1) {
        return new StateMachineBuilder()
                .state(ArmState.Hover)
                .loop(() -> {
                    currentState = ArmState.Hover;
                    leftShooter.setPower(0);
                    rightShooter.setPower(0);
                    spinIntake.setPower(0);

                    prevB = gamepad1.b;
                    prevRB = gamepad1.right_bumper;
                })
                .transition(() -> risingEdge(gamepad1.b, prevB), ArmState.Intake)

                .state(ArmState.Intake)
                .loop(() -> {
                    currentState = ArmState.Intake;
                    leftShooter.setPower(0);
                    rightShooter.setPower(0);
                    spinIntake.setPower(1);

                    prevB = gamepad1.b;
                    prevRB = gamepad1.right_bumper;
                })
                .transition(() -> risingEdge(gamepad1.right_bumper, prevRB), ArmState.Loaded)
                .state(ArmState.Loaded)
                .loop(()->{
                    spinIntake.setPower(0);
                    idle1.setPosition(1);
                    idle2.setPosition(1);
                    idle3.setPosition(1);
                    idle4.setPosition(1);
                    leftShooter.setPower(1);
                    rightShooter.setPower(1);
                    prevB = gamepad1.b;
                    prevRB = gamepad1.right_bumper;
                })
                .transition(() -> risingEdge(gamepad1.right_bumper, prevRB), ArmState.Shoot1)
                .state(ArmState.Shoot1)
                .loop(() -> {
                    currentState = ArmState.Shoot1;
                    leftShooter.setPower(1);
                    rightShooter.setPower(1);
                    spinIntake.setPower(0);

                    // Update button states
                    prevB = gamepad1.b;
                    prevRB = gamepad1.right_bumper;
                })
                .transition(() -> risingEdge(gamepad1.b, prevB), ArmState.Shoot2)
                .state(ArmState.Shoot2)
                .loop(() -> {
                    currentState = ArmState.Shoot2;
                    leftShooter.setPower(1);
                    rightShooter.setPower(1);
                    spinIntake.setPower(0);

                    // Update button states
                    prevB = gamepad1.b;
                    prevRB = gamepad1.right_bumper;
                })
                .transition(() -> risingEdge(gamepad1.b, prevB), ArmState.Shoot3)
                .state(ArmState.Shoot3)
                .loop(() -> {
                    currentState = ArmState.Shoot3;
                    leftShooter.setPower(1);
                    rightShooter.setPower(1);
                    spinIntake.setPower(0);

                    // Update button states
                    prevB = gamepad1.b;
                    prevRB = gamepad1.right_bumper;
                })
                .transition(() -> risingEdge(gamepad1.b, prevB), ArmState.Hover)
                .build();
    }
}

