package org.firstinspires.ftc.teamcode.subsystem;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.TeamConstants;

public class Transfer implements Subsystem {

    private DcMotorEx transfer;
    private DistanceSensor topSensor;

    public enum State {
        IDLE,
        FEEDING,
        BALL_HELD_AT_TOP,
        SHOOTING
    }

    private State state = State.IDLE;

    public Transfer(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
        topSensor = hardwareMap.get(DistanceSensor.class, "TransferSensor");
    }

    @Override
    public void init() {
        //Uselss init as nothing needs to be here due to it implementing methods from the Subsystem interface
    }

    private boolean ballAtTop() {
        return topSensor.getDistance(DistanceUnit.CM) < TeamConstants.BALL_THRESHOLD_CM;
    }

    public void startFeeding() {
        state = State.FEEDING;
    }

    public void startShooting() {
        state = State.SHOOTING;
    }

    public void stop() {
        state = State.IDLE;
        transfer.setPower(TeamConstants.TRANSFER_CLOSED);
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2) {

        switch (state) {

            case IDLE:
                transfer.setPower(TeamConstants.TRANSFER_CLOSED);
                break;

            case FEEDING:
                if (ballAtTop()) {
                    transfer.setPower(TeamConstants.TRANSFER_CLOSED);
                    state = State.BALL_HELD_AT_TOP;
                } else {
                    transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
                }
                break;

            case BALL_HELD_AT_TOP:
                transfer.setPower(TeamConstants.TRANSFER_CLOSED);
                break;

            case SHOOTING:
                transfer.setPower(TeamConstants.TRANSFER_IN_POWER);

                // Detect ball leaving
                if (!ballAtTop()) {
                    state = State.FEEDING;
                }
                break;
        }
    }
}
