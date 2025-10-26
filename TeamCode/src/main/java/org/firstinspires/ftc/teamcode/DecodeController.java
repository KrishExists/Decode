package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.arcrobotics.ftclib.util.InterpLUT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    public States state;
    private double power;
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    InterpLUT lut;
    public DecodeController(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class,"Intake");
        shooter = hardwareMap.get(DcMotorEx.class,"shooter");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lut = new InterpLUT();

//Adding each val with a key
        lut.add(30, 1);
        lut.add(2.7, .5);
        lut.add(3.6, 0.75);
        lut.add(4.1, 0.9);
        lut.add(5, 1);
//generating final equation
        lut.createLUT();
        initAprilTag();

    }
    public void controllerInit(){
        state= States.Intake;
    }
    public enum States{
        Intake,
        Rest,
        Outake,
        Calculate
    }
    public StateMachine shooterMachine(Gamepad gamepad1, Gamepad gamepad2){
        return new StateMachineBuilder()
                .state(States.Intake)
                .loop(()->{
                    intake.setPower(1);
                    shooter.setPower(0.01);
                })
                .transition(() -> {
                    boolean fallingEdge = gamepad2.backWasPressed();
                    return fallingEdge;
                }, States.Calculate)
                .onEnter(()->{
                    visionPortal.resumeStreaming();
                })
                .state(States.Calculate)
                .loop(()->{
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                    // Step through the list of detections and display info for each one.
                    for (AprilTagDetection detection : currentDetections) {
                        if(detection.id <=23){
                            continue;
                        }else{
                            double range = detection.ftcPose.range;
                            //power = lut.get(range);
                        }
                    }
                        })
                .onExit(()->{
                    visionPortal.stopStreaming();
                })
                .states(States.Outake)
                .loop(()->{
                    intake.setPower(0.5);
                    //shooter.setPower(power); use this when we actually can
                    shooter.setPower(0.65);
                })
                .transition(()->{
                    boolean fallingEdge = gamepad2.backWasPressed();
                    return fallingEdge;
                },States.Rest)
                .states(States.Rest)
                .loop(()->{
                    intake.setPower(0);
                    shooter.setPower(0);
                })
                .transition(()->{
                    boolean fallingEdge = gamepad2.backWasPressed();
                    return fallingEdge;
                },States.Intake)

                .build();


    }
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

}
