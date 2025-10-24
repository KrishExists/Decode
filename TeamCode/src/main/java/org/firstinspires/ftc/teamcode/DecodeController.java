package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

public class DecodeController {
    private DcMotorEx shooter;
    private DcMotorEx intake;
    public States state;
    public DecodeController(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        shooter = hardwareMap.get(DcMotorEx.class,"shooter");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    public StateMachine armMachine(Gamepad gamepad1, Gamepad gamepad2){
        return new StateMachineBuilder()
                .state(States.Intake)
                .loop(()->{
                    intake.setPower(1);
                    shooter.setPower(0.01);
                })
                .transition(() -> {
                    boolean fallingEdge = gamepad2.backWasPressed();
                    return fallingEdge;
                }, States.Outake)

                .build();


    }

}
