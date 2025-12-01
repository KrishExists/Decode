package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class Hardware {
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    public DcMotor intake;
    public DcMotorEx outtake, outtake2;
    public Servo linkage, blocker;
    public WebcamName webcam;


    private final HardwareMap hw;


    public Hardware(HardwareMap hardwareMap) {
        this.hw = hardwareMap;
    }


    public void init() {
        frontLeftMotor = hw.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hw.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hw.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hw.get(DcMotor.class, "backRightMotor");


        intake = hw.get(DcMotor.class, "Intake");
        outtake = hw.get(DcMotorEx.class, "Outtake");
        outtake2 = hw.get(DcMotorEx.class, "Outtake2");


        linkage = hw.get(Servo.class, "Linkage");
        blocker = hw.get(Servo.class, "Blocker");


        webcam = hw.get(WebcamName.class, "Webcam 1");


// Motor directions & zero power behavior
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);


        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        linkage.setPosition(0.92);
        blocker.setPosition(0.6);
    }
}