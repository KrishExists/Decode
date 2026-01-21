package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.TeamConstants;
import org.firstinspires.ftc.teamcode.util.StateMachine;

public class Intake implements Subsystem{

    ElapsedTime time;
    private boolean happend;
    private Servo blocker;

    public enum IntakeState {
        RUNSLOW, RAPOD_FAR,RAPOD_CLOSE, IntakeNEXT, REST, INTAKE_LAST, SpeedMid,SpeedFar, AUTORPMRED
    }

    // Hardware
    private final DcMotor intake;
    private final Servo linkage;
    public static boolean far;
    public static boolean close;

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

    public static boolean next;

    public Intake(HardwareMap hw, Telemetry t, Outtake shooter) {
        this.telemetry = t;
        this.shooter = shooter;
        next = false;
        far = false;
        close= false;

        // Map hardware
        intake = hw.get(DcMotor.class, "Intake");
        linkage = hw.get(Servo.class, "Linkage");
        transfer = hw.get(DcMotorEx.class, "Transfer");
        blocker = hw.get(Servo.class, "blocker");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        blocker.setPosition(0.5);

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
        intake.setPower(TeamConstants.INTAKE_DEFAULT_POWER);
        linkage.setPosition(TeamConstants.LINKAGE_REST);
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
        else if (gamepad2.right_bumper||next){
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
        if(far){
            setState(IntakeState.SpeedFar);
            shooting = true;
        }
        if(close){
            setState(IntakeState.SpeedMid);
            shooting = true;
        }

        long t = sm.timeInStateMs();

        // -----------------------------
        // STATE LOGIC
        // -----------------------------
        switch (sm.getState()) {

            case INTAKE_LAST:
                shooter.setPower(TeamConstants.SHOOTER_CLOSED);

                intake.setPower(TeamConstants.INTAKE_IN_POWER);
                transfer.setPower(0);
                break;

            case IntakeNEXT:

                intake.setPower(TeamConstants.INTAKE_IN_POWER);
                shooter.setPower(TeamConstants.SHOOTER_CLOSED);


                linkage.setPosition(TeamConstants.LINKAGE_REST);
                transfer.setPower(TeamConstants.TRANSFER_IN_POWER);


                break;


            case RUNSLOW:
                intake.setPower(TeamConstants.intakeReversed);
                transfer.setPower(TeamConstants.TRANSFER_REV);
                shooter.setPower(TeamConstants.SLIGHT_REVERSE_OUTTAKE);

                break;

            case SpeedFar:
                intake.setPower(TeamConstants.INTAKE_EVEN_POWER);
                linkage.setPosition(TeamConstants.LINKAGE_SHOOT);
                shooter.spinToRpm(TeamConstants.SHOOTER_FAR_RPM);
                transfer.setPower(-1);
                if(gamepad2.right_bumper||next){
                    setState(IntakeState.RAPOD_FAR);
                    shooting = false;
                    far = false;

                }
                break;
            case SpeedMid:
                intake.setPower(TeamConstants.INTAKE_EVEN_POWER);
                linkage.setPosition(TeamConstants.LINKAGE_SHOOT);
                shooter.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
                transfer.setPower(-1);
                if(gamepad2.right_bumper||next){
                    setState(IntakeState.RAPOD_CLOSE);
                    intake.setPower(TeamConstants.INTAKE_DEFAULT_POWER);
                    transfer.setPower(TeamConstants.TRANSFER_CLOSED);
                    shooting = false;
                    close = false;
                }
                break;


            case RAPOD_CLOSE:
                blocker.setPosition(TeamConstants.BLOCKER_OPEN);
                if(shooter.getRPM()>TeamConstants.SHOOTER_MID_RPM-100){
                    happend = true;
                }
                if (happend){
                    shooter.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
                    intake.setPower(TeamConstants.INTAKE_IN_POWER);
                    transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
                }
                break;
            case RAPOD_FAR:
                blocker.setPosition(TeamConstants.BLOCKER_OPEN);
                if(shooter.getRPM()>TeamConstants.SHOOTER_FAR_RPM-100){
                    happend = true;
                }
                if(happend){
                    shooter.spinToRpm(TeamConstants.SHOOTER_FAR_RPM);
                    intake.setPower(TeamConstants.INTAKE_IN_POWER);
                    transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
                }
                break;

            case REST:
                happend = false;

                intake.setPower(TeamConstants.INTAKE_DEFAULT_POWER);
                shooter.stop();
                transfer.setPower(TeamConstants.TRANSFER_CLOSED);
                linkage.setPosition(TeamConstants.LINKAGE_REST);
                break;
            default:
                happend = false;
                blocker.setPosition(TeamConstants.BLOCKER_CLOSE);

                intake.setPower(TeamConstants.INTAKE_DEFAULT_POWER);
                shooter.stop();
                transfer.setPower(TeamConstants.TRANSFER_CLOSED);
                linkage.setPosition(TeamConstants.LINKAGE_REST);
                break;
        }

        telemetry.addData("Intake State", sm.getState());
        telemetry.addData("Time in State", t);
    }


}
