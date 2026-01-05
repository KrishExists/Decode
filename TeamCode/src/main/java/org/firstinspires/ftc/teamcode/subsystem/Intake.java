package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.StateMachine;

public class Intake implements Subsystem{

    ElapsedTime time;

    public enum IntakeState {
        RUNSLOW, RAPOD_FAR,RAPOD_CLOSE, IntakeNEXT, REST, INTAKE_LAST, SpeedMid,SpeedFar, AUTORPMRED
    }

    // Hardware
    private final DcMotor intake;
    private final Servo linkage;

    private double goodRPM;
    ElapsedTime timer = new ElapsedTime();

    public DcMotorEx transfer;

    private final Outtake shooter;
    private final boolean rapid = false;
    public boolean AtRPM = false;
    private final Telemetry telemetry;
    private boolean shooting = false;

    private final boolean transfer123 = false;




    // State machine
    private final StateMachine<IntakeState> sm;

    // Gamepad updated from OpMode
    private Gamepad gamepad;

    public Intake(HardwareMap hw, Telemetry t, Outtake shooter) {
        this.telemetry = t;
        this.shooter = shooter;

        // Map hardware
        intake = hw.get(DcMotor.class, "Intake");
        linkage = hw.get(Servo.class, "Linkage");
        transfer = hw.get(DcMotorEx.class, "Transfer");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize state machine
        sm = new StateMachine<>(IntakeState.REST);
        timerReset();
        timerStart();
    }

    public void setGamepad(Gamepad g) {
        this.gamepad = g;
    }

    public void setPower(double power) {
        intake.setPower(power);
        transfer.setPower(power);
    }

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
        linkage.setPosition(Constants.LINKAGE_REST);
    }





    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad2 == null) return;

        // -----------------------------
        // MAP GAMEPAD INPUT TO STATES
        // -----------------------------
        //Button controls
        //Dpad up for first ball. dpad left for second and third
        //Dpad right to cancel
        //Y for auto speed dont like see if i fix
        //Right bumper for outtake
        //Bottom 2 for outtake mid and outtake far just speeding up


        else if(gamepad2.dpad_up) setState(IntakeState.IntakeNEXT);
        else if (gamepad2.dpad_left) setState(IntakeState.INTAKE_LAST);
        else if (gamepad2.dpad_right) {
            setState(IntakeState.REST);
            shooting = false;
        }
        else if (gamepad2.dpad_down) setState(IntakeState.RUNSLOW);
        else if (gamepad2.right_bumper){
            //IS CURCIAL PLEASE DO NOT DELETE. I REPEAT DO NOT DELETE. IS HELPING US PLEASE MURDUKUR
        }

        else if (gamepad2.a){
            setState(IntakeState.SpeedMid);
            shooting = true;
        }
        else if (gamepad2.b) {
            setState(IntakeState.SpeedFar);
            shooting = true;
        }

        else if(gamepad2.y) {
            setState(IntakeState.AUTORPMRED);
            shooting = true;
        }
        else if (shooting){
            //do nothing AGAIN IS CRUCIAL MURDURKUR
        }
        else setState(IntakeState.REST);


        long t = sm.timeInStateMs();

        // -----------------------------
        // STATE LOGIC
        // -----------------------------
        switch (sm.getState()) {

            case INTAKE_LAST:
                shooter.setPower(Constants.SLIGHT_REVERSE_OUTTAKE);

                intake.setPower(Constants.INTAKE_IN_POWER);
                break;

            case IntakeNEXT:

                intake.setPower(Constants.INTAKE_IN_POWER);
                shooter.setPower(Constants.SLIGHT_REVERSE_OUTTAKE);


                linkage.setPosition(Constants.LINKAGE_REST);
                transfer.setPower(Constants.TRANSFER_IN_POWER);
                break;


//            case AUTORPMRED:
//                intake.setPower(0.6);
//                linkage.setPosition(0.5);
//                //shooter.spinToRpm(2996);
//                goodRPM = 8.980589* TeleopRed.distanceFromGoal() + 1847.499;
//                intake.setPower(0.8);
//                transfer.setPower(0.8);
//                shooter.spinToRpm(goodRPM);
//                if(gamepad2.right_bumper){
//                    if(true){
//                        setState(IntakeState.OUTTAKE_FAR);
//                        intake.setPower(0);
//                        transfer.setPower(0);
//                    }else{
//                        setState(IntakeState.RAPOD_FAR);
//                    }
//                    ;
//                    shooting = false;
//
//                }
//                break;


            case RUNSLOW:
                intake.setPower(-1);
                transfer.setPower(1);
                shooter.setPower(-0.5);
                break;

            case SpeedFar:
                intake.setPower(Constants.INTAKE_EVEN_POWER);
                linkage.setPosition(Constants.LINKAGE_MID);
                shooter.spinToRpm(Constants.SHOOTER_FAR_RPM);
                intake.setPower(Constants.INTAKE_IN_POWER);
                transfer.setPower(Constants.TRANSFER_EVEN);
                if(gamepad2.right_bumper){
                    setState(IntakeState.RAPOD_FAR);
                    shooting = false;

                }
                break;
            case SpeedMid:
                intake.setPower(Constants.INTAKE_EVEN_POWER);
                linkage.setPosition(Constants.LINKAGE_MID);

                shooter.spinToRpm(Constants.SHOOTER_MID_RPM);
                transfer.setPower(Constants.TRANSFER_EVEN);
                if(gamepad2.right_bumper){
                    setState(IntakeState.RAPOD_CLOSE);
                    intake.setPower(0);
                    transfer.setPower(0);
                    shooting = false;
                }
                break;


            case RAPOD_CLOSE:
               shooter.spinToRpm(Constants.SHOOTER_MID_RPM);
               intake.setPower(Constants.INTAKE_IN_POWER);
               transfer.setPower(Constants.TRANSFER_IN_POWER);
                break;
            case RAPOD_FAR:
                shooter.spinToRpm(Constants.SHOOTER_FAR_RPM);
                intake.setPower(Constants.INTAKE_IN_POWER);
                transfer.setPower(Constants.TRANSFER_IN_POWER);
                break;

            case REST:

            default:
                intake.setPower(0);
                shooter.stop();
                transfer.setPower(Constants.TRANSFER_CLOSED);
                linkage.setPosition(Constants.LINKAGE_REST);
                break;
        }

        telemetry.addData("Intake State", sm.getState());
        telemetry.addData("Time in State", t);
    }


}
