package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class PIDF_Arm extends OpMode { // Tune values after the end effector is on
    PIDController controller;
    public static double kp = 0.002; // Proportional gain
    public static double ki=0; // Integral gain
    public static double kd=0.00001; // Derivative gain
    public static double kcos=0.07;//0.07
    private double integral; // Integral sum
    private double previousError;// Previous error
    public static double target = 0;
    private DcMotorEx armMotorRight;
    private DcMotorEx armMotorLeft;
    private Servo wristLeft;
    private Servo wristRight;
    private Servo pivotServo;
    private Servo clawServo;
    private Servo clawRotate;
    @Override
    public void init() {
        controller = new PIDController(kp,ki,kd,kcos);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armMotorRight = hardwareMap.get(DcMotorEx.class, "armMotorRight");
        armMotorLeft = hardwareMap.get(DcMotorEx.class, "armMotorLeft");
        armMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
        armMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        wristRight = hardwareMap.get(Servo.class, "wristRight");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");

        // Reversed wrist servos
        wristLeft.setDirection(Servo.Direction.REVERSE);
        wristRight.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
        wristLeft.setPosition(0);
        pivotServo.setPosition(0.45);
        clawServo.setPosition(0);
        clawRotate.setPosition(0);
        int armPos = armMotorRight.getCurrentPosition();
        int armPosLeft = armMotorLeft.getCurrentPosition();
        controller.setCoeffs(kp, ki, kd, kcos);
        double power = controller.calculate(target, armPos);
        armMotorRight.setPower(power);//DO NOT CHANGE FROM NEGATIVE TO POSITIVE OR ELSE MOTORS GET BURNT
        armMotorLeft.setPower(power); //DO NOT CHANGE FROM POSITIVE TO NEGATIVE OR ELSE MOTORS GET BURNT
        telemetry.addData("pos: ", armPos);
        telemetry.addData("pos: ", armPosLeft);
        telemetry.addData("target: ", target);
        telemetry.addData("Power for ArmMotorRight:", armMotorRight.getPower());
        telemetry.addData("Power for ArmMotorLeft:", armMotorLeft.getPower());
        telemetry.update();
    }
}
