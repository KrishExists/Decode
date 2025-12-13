//package org.firstinspires.ftc.teamcode.subsystem;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Const;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.util.Constants;
//import org.firstinspires.ftc.teamcode.util.StateMachine;
//
//public class Intake implements Subsystem{
//
//    ElapsedTime time;
//
//    public enum IntakeState {
//        INTAKE, OUTTAKE, RUNSLOW, OUTTAKE1, OUTTAKE_FAR, OUTTAKE_MID, TRANSFER, IntakeNEXT, REST, INTAKE_LAST
//    }
//
//    // Hardware
//    private DcMotor intake;
//    private Servo blocker;
//    private Servo linkage;
//
//     ElapsedTime timer = new ElapsedTime();
//
//    private DcMotorEx transfer;
//
//    private final Outtake shooter;
//
//    public boolean AtRPM = false;
//    private final Telemetry telemetry;
//
//
//
//    // State machine
//    private final StateMachine<IntakeState> sm;
//
//    // Gamepad updated from OpMode
//    private Gamepad gamepad;
//
//    public Intake(HardwareMap hw, Telemetry t, Outtake shooter) {
//        this.telemetry = t;
//        this.shooter = shooter;
//
//        // Map hardware
//        intake = hw.get(DcMotor.class, "Intake");
//        blocker = hw.get(Servo.class, "Blocker");
//        linkage = hw.get(Servo.class, "Linkage");
//        transfer = hw.get(DcMotorEx.class, "Transfer");
//
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        // Initialize state machine
//        sm = new StateMachine<>(IntakeState.REST);
//    }
//
//    public void setGamepad(Gamepad g) {
//        this.gamepad = g;
//    }
//
//    public void setPower(double power) { intake.setPower(power); }
//
//    public void setState(IntakeState s) { sm.setState(s); }
//    public IntakeState getState() { return sm.getState(); }
//    public void timerReset(){
//        timer.reset();
//    }
//    public void timerStart(){
//        timer.startTime();
//    }
//
//
//    public void init() {
//        intake.setPower(0);
//        blocker.setPosition(Constants.BLOCKER_CLOSE);
//        linkage.setPosition(Constants.LINKAGE_REST);
//    }
//
//
//
//
//
//    @Override
//    public void update(Gamepad gamepad2) {
//        if (gamepad2 == null) return;
//
//        // -----------------------------
//        // MAP GAMEPAD INPUT TO STATES
//        // -----------------------------
//        if (gamepad2.dpad_down) setState(IntakeState.INTAKE);
//        if(gamepad2.dpad_up) setState(IntakeState.IntakeNEXT);
//        else if (gamepad2.dpad_left) setState(IntakeState.INTAKE_LAST);
//
//        else if (gamepad2.left_trigger > 0.2) setState(IntakeState.OUTTAKE);
//        else if (gamepad2.dpad_down) setState(IntakeState.RUNSLOW);
//        else if (gamepad2.a) setState(IntakeState.OUTTAKE1);
//        else if (gamepad2.x) setState(IntakeState.OUTTAKE_FAR);
//        else if (gamepad2.y) setState(IntakeState.OUTTAKE_MID);
//        else setState(IntakeState.REST);
//
//        long t = sm.timeInStateMs();
//
//        // -----------------------------
//        // STATE LOGIC
//        // -----------------------------
//        switch (sm.getState()) {
//
//            case INTAKE:
//                intake.setPower(Constants.INTAKE_IN_POWER);
//                shooter.stop();
//                linkage.setPosition(Constants.LINKAGE_REST);
//                blocker.setPosition(Constants.BLOCKER_OPEN);
//                transfer.setPower(Constants.TRANSFER_IN_POWER);
//                break;
//            case INTAKE_LAST:
//                intake.setPower(Constants.INTAKE_IN_POWER);
//                break;
//
//            case IntakeNEXT:
//                intake.setPower(Constants.INTAKE_IN_POWER);
//                shooter.stop();
//                linkage.setPosition(Constants.LINKAGE_REST);
//                blocker.setPosition(Constants.BLOCKER_OPEN);
//                transfer.setPower(Constants.TRANSFER_IN_POWER);
//                break;
//
//            case TRANSFER:
//                timer.reset();
//                blocker.setPosition(Constants.BLOCKER_CLOSE);
//                intake.setPower(Constants.INTAKE_REVERSE_KEEP);
//                transfer.setPower(Constants.TRANSFER_REV);
//                if (timer.seconds()>=4){
//                    transfer.setPower(0);
//                    intake.setPower(0);
//
//                }
//
//                shooter.stop();
//                break;
//
//            case RUNSLOW:
//                transfer.setPower(1.0);
//                intake.setPower(-0.5);
//                shooter.reverse();
//                linkage.setPosition(Constants.LINKAGE_REST);
//                break;
//
//            case OUTTAKE1:
//                blocker.setPosition(Constants.BLOCKER_OPEN);
//                if (t < 1500) {
//                    shooter.spinToRpm(2000);
//                    intake.setPower(0);
//                    linkage.setPosition(Constants.LINKAGE_REST);
//                } else if (t < 3300) {
//                    shooter.spinToRpm(2400);
//                    intake.setPower(0);
//                    linkage.setPosition(Constants.LINKAGE_SHOOT);
//                } else {
//                    shooter.spinToRpm(2400);
//                    intake.setPower(Constants.INTAKE_FEED_POWER);
//                    linkage.setPosition(Constants.LINKAGE_SHOOT);
//                }
//                break;
//
//            case OUTTAKE:
//                blocker.setPosition(Constants.BLOCKER_OPEN);
//                if (t < 700) {
//                    intake.setPower(-0.500);
//                    shooter.reverse();
//                    linkage.setPosition(Constants.LINKAGE_REST);
//                } else if (t < 2500) {
//                    shooter.spinToRpm(2700);
//                    intake.setPower(0);
//                } else if (t < 3000) {
//                    shooter.spinToRpm(2700);
//                    linkage.setPosition(Constants.LINKAGE_SHOOT);
//                } else if (t < 4000) {
//                    shooter.spinToRpm(2700);
//                    intake.setPower(Constants.INTAKE_FEED_POWER);
//                } else {
//                    shooter.spinToRpm(2700);
//                    intake.setPower(Constants.INTAKE_FEED_POWER);
//                }
//                break;
//
//            case OUTTAKE_MID:
//                blocker.setPosition(Constants.BLOCKER_OPEN);
//                if (t < 500) {
//                    linkage.setPosition(Constants.LINKAGE_MID);
//                } else {
//                    shooter.spinToRpm(4000);
//                    if (shooter.atSpeed(3100, 3600)) {
//                            intake.setPower(Constants.INTAKE_FEED_POWER);
//                        transfer.setPower(Constants.TRANSFER_IN_POWER);
//                    } else {
//                        intake.setPower(0);
//                        transfer.setPower(Constants.TRANSFER_CLOSED);
//                    }
//                }
//                break;
//
//            case OUTTAKE_FAR:
//                blocker.setPosition(Constants.BLOCKER_OPEN);
//                if (t < 500) {
//                    linkage.setPosition(Constants.LINKAGE_MID);
//                } else {
//                    shooter.spinToRpm(6000);
//                    if (shooter.atSpeed(3700, 6000)) {
//                          intake.setPower(Constants.INTAKE_FEED_POWER);
//                        transfer.setPower(Constants.TRANSFER_IN_POWER);
//                    } else {
//                        intake.setPower(0);
//                        transfer.setPower(Constants.TRANSFER_CLOSED);
//                    }
//                }
//                break;
//
//            case REST:
//            default:
//                intake.setPower(0);
//                shooter.stop();
//                blocker.setPosition(Constants.BLOCKER_CLOSE);
//                transfer.setPower(Constants.TRANSFER_CLOSED);
//                linkage.setPosition(Constants.LINKAGE_REST);
//                break;
//        }
//
//        telemetry.addData("Intake State", sm.getState());
//        telemetry.addData("Time in State", t);
//        telemetry.update();
//    }
////    public void resetTime(){
////        public void .resetTime();
////    }
//}

package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.StateMachine;

public class Intake implements Subsystem{

    ElapsedTime time;

    public enum IntakeState {
        INTAKE, OUTTAKE, RUNSLOW, OUTTAKE1, OUTTAKE_FAR, OUTTAKE_MID, TRANSFER, IntakeNEXT, REST, INTAKE_LAST, SpeedMid,SpeedFar
    }

    // Hardware
    private DcMotor intake;
    private Servo blocker;
    private Servo linkage;

    ElapsedTime timer = new ElapsedTime();

    private DcMotorEx transfer;

    private final Outtake shooter;

    public boolean AtRPM = false;
    private final Telemetry telemetry;
    private boolean shooting = false;




    // State machine
    private final StateMachine<IntakeState> sm;

    // Gamepad updated from OpMode
    private Gamepad gamepad;

    public Intake(HardwareMap hw, Telemetry t, Outtake shooter) {
        this.telemetry = t;
        this.shooter = shooter;

        // Map hardware
        intake = hw.get(DcMotor.class, "Intake");
        blocker = hw.get(Servo.class, "Blocker");
        linkage = hw.get(Servo.class, "Linkage");
        transfer = hw.get(DcMotorEx.class, "Transfer");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize state machine
        sm = new StateMachine<>(IntakeState.REST);
    }

    public void setGamepad(Gamepad g) {
        this.gamepad = g;
    }

    public void setPower(double power) { intake.setPower(power); }

    public void setState(IntakeState s) { sm.setState(s); }
    public IntakeState getState() { return sm.getState(); }
    public void timerReset(){
        timer.reset();
    }
    public void timerStart(){
        timer.startTime();
    }


    public void init() {
        intake.setPower(0);
        blocker.setPosition(Constants.BLOCKER_CLOSE);
        linkage.setPosition(Constants.LINKAGE_REST);
    }





    @Override
    public void update(Gamepad gamepad2) {
        if (gamepad2 == null) return;

        // -----------------------------
        // MAP GAMEPAD INPUT TO STATES
        // -----------------------------
        if (gamepad2.dpad_down) setState(IntakeState.INTAKE);
        if(gamepad2.dpad_up) setState(IntakeState.IntakeNEXT);
        else if (gamepad2.dpad_left) setState(IntakeState.INTAKE_LAST);

        else if (gamepad2.dpad_down) setState(IntakeState.RUNSLOW);
        else if (gamepad2.right_bumper){
        }
        else if (gamepad2.a){
            setState(IntakeState.SpeedMid);
            shooting = true;
        }
        else if (gamepad2.b) {
            setState(IntakeState.SpeedFar);
            shooting = true;
        }
        else if (shooting){
            //do nothing
        }
        else setState(IntakeState.REST);

        long t = sm.timeInStateMs();

        // -----------------------------
        // STATE LOGIC
        // -----------------------------
        switch (sm.getState()) {

            case INTAKE:
                intake.setPower(Constants.INTAKE_IN_POWER);
                shooter.stop();
                linkage.setPosition(Constants.LINKAGE_REST);
                blocker.setPosition(Constants.BLOCKER_OPEN);
                transfer.setPower(Constants.TRANSFER_IN_POWER);
                break;
            case INTAKE_LAST:
                intake.setPower(Constants.INTAKE_IN_POWER);
                break;

            case IntakeNEXT:
                intake.setPower(Constants.INTAKE_IN_POWER);
                shooter.stop();
                linkage.setPosition(Constants.LINKAGE_REST);
                blocker.setPosition(Constants.BLOCKER_OPEN);
                transfer.setPower(Constants.TRANSFER_IN_POWER);
                break;

            case TRANSFER:
                timer.reset();
                blocker.setPosition(Constants.BLOCKER_CLOSE);
                intake.setPower(Constants.INTAKE_REVERSE_KEEP);
                transfer.setPower(Constants.TRANSFER_REV);
                if (timer.seconds()>=4){
                    transfer.setPower(0);
                    intake.setPower(0);

                }

                shooter.stop();
                break;

            case RUNSLOW:
                transfer.setPower(1.0);
                intake.setPower(-0.5);
                shooter.reverse();
                linkage.setPosition(Constants.LINKAGE_REST);
                break;

            case OUTTAKE1:
                blocker.setPosition(Constants.BLOCKER_OPEN);
                if (t < 1500) {
                    shooter.spinToRpm(2000);
                    intake.setPower(0);
                    linkage.setPosition(Constants.LINKAGE_REST);
                } else if (t < 3300) {
                    shooter.spinToRpm(2400);
                    intake.setPower(0);
                    linkage.setPosition(Constants.LINKAGE_SHOOT);
                } else {
                    shooter.spinToRpm(2400);
                    intake.setPower(Constants.INTAKE_FEED_POWER);
                    linkage.setPosition(Constants.LINKAGE_SHOOT);
                }
                break;
            case SpeedFar:
                intake.setPower(0.6);
                linkage.setPosition(Constants.LINKAGE_MID);
                shooter.spinToRpm(6000);
                if(gamepad2.right_bumper){
                    setState(IntakeState.OUTTAKE_FAR);
                    shooting = false;
                }
                break;
            case SpeedMid:
                intake.setPower(0.6);
                linkage.setPosition(Constants.LINKAGE_MID);
                shooter.spinToRpm(4000);
                if(gamepad2.right_bumper){
                    setState(IntakeState.OUTTAKE_MID);
                    shooting = false;
                }
                break;
            case OUTTAKE:
                blocker.setPosition(Constants.BLOCKER_OPEN);
                if (t < 700) {
                    intake.setPower(-0.500);
                    shooter.reverse();
                    linkage.setPosition(Constants.LINKAGE_REST);
                } else if (t < 2500) {
                    shooter.spinToRpm(2700);
                    intake.setPower(0);
                } else if (t < 3000) {
                    shooter.spinToRpm(2700);
                    linkage.setPosition(Constants.LINKAGE_SHOOT);
                } else if (t < 4000) {
                    shooter.spinToRpm(2700);
                    intake.setPower(Constants.INTAKE_FEED_POWER);
                } else {
                    shooter.spinToRpm(2700);
                    intake.setPower(Constants.INTAKE_FEED_POWER);
                }
                break;

            case OUTTAKE_MID:
                blocker.setPosition(Constants.BLOCKER_OPEN);
                if (t < 0) {
                    linkage.setPosition(Constants.LINKAGE_MID);

                } else {
                    shooter.spinToRpm(3700);
                    if (shooter.atSpeed(3100, 4100)) {
                        intake.setPower(Constants.INTAKE_FEED_POWER);
                        transfer.setPower(Constants.TRANSFER_IN_POWER);
                    } else {
                        intake.setPower(0);
                        transfer.setPower(Constants.TRANSFER_CLOSED);
                    }
                }
                break;

            case OUTTAKE_FAR:
                blocker.setPosition(Constants.BLOCKER_OPEN);
                if (t < 0) {
                    linkage.setPosition(Constants.LINKAGE_MID);
                } else {
                    shooter.spinToRpm(6000);
                    if (shooter.atSpeed(3700, 6000)) {
                        intake.setPower(Constants.INTAKE_FEED_POWER);
                        transfer.setPower(Constants.TRANSFER_IN_POWER);
                    } else {
                        intake.setPower(0);
                        transfer.setPower(Constants.TRANSFER_CLOSED);
                    }
                }
                break;

            case REST:
            default:
                intake.setPower(0);
                shooter.stop();
                blocker.setPosition(Constants.BLOCKER_CLOSE);
                transfer.setPower(Constants.TRANSFER_CLOSED);
                linkage.setPosition(Constants.LINKAGE_REST);
                break;
        }

        telemetry.addData("Intake State", sm.getState());
        telemetry.addData("Time in State", t);
        telemetry.update();
    }
//    public void resetTime(){
//        public void .resetTime();
//    }
}
