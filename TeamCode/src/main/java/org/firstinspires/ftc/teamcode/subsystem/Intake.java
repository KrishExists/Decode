package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
    InterpLUT interpLUT;


    // State machine
    private final StateMachine<IntakeState> sm;

    // Gamepad updated from OpMode
    private Gamepad gamepad;

    public static boolean next;
    private Follower follower;
    private Pose goal;
    public Intake(HardwareMap hw, Telemetry t, Outtake shooter, Follower follower) {
        this.telemetry = t;
        this.shooter = shooter;
        this.follower = follower;
        next = false;
        far = false;
        close= false;
        interpLUT = new InterpLUT();
        interpLUT.add(0, 0);
        interpLUT.add(62, 2100);
        interpLUT.add(69, 2250);
        interpLUT.add(81, 2375);
        interpLUT.add(100, 2480);
        interpLUT.add(121, 2925);
        interpLUT.add(138, 3300);
        interpLUT.createLUT();
        // Map hardware
        intake = hw.get(DcMotor.class, "Intake");
        linkage = hw.get(Servo.class, "Linkage");
        transfer = hw.get(DcMotorEx.class, "Transfer");
        blocker = hw.get(Servo.class, "blocker");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        blocker.setPosition(0.5);
        goal = new Pose(133,133,0);

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


        else if(gamepad2.right_trigger>0.2) setState(IntakeState.IntakeNEXT);
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

        else if(gamepad2.left_trigger>0.2) {
            setState(IntakeState.AUTORPMRED);
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

            case IntakeNEXT:
                shooter.setPower(TeamConstants.SHOOTER_CLOSED);
                blocker.setPosition(TeamConstants.BLOCKER_CLOSE);

                intake.setPower(TeamConstants.INTAKE_IN_POWER);

                transfer.setPower(-1);
                break;

            case AUTORPMRED:
                double distance = follower.getPose().distanceFrom(goal);
                if(distance > 100)
                    distance = 99;
//                double rpm = 9.43976 * distance +1500.948;//linear regression model
                double rpm = interpLUT.get(distance);
                shooter.spinToRpm(rpm);
                if(shooter.getRPM()>rpm-25&&shooter.getRPM()<rpm + 100){
                    happend = true;

                }
                if (happend){
                    intake.setPower(TeamConstants.INTAKE_IN_POWER);
                    transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
                    blocker.setPosition(0.58);
                }else{
                    intake.setPower(0);
                    transfer.setPower(0);
                }
                break;
            case RUNSLOW:
                intake.setPower(TeamConstants.intakeReversed);
                transfer.setPower(TeamConstants.TRANSFER_REV);
                shooter.setPower(TeamConstants.SLIGHT_REVERSE_OUTTAKE);

                break;

            case SpeedFar:
                //kp is 0.07
                //kd is 0.00001
                intake.setPower(0);
                linkage.setPosition(TeamConstants.LINKAGE_REST);
                shooter.spinToRpm(TeamConstants.SHOOTER_FARAUTO_RPM);
                blocker.setPosition(TeamConstants.BLOCKER_OPEN);
                transfer.setPower(0);
                if(gamepad2.right_bumper||next){
                    setState(IntakeState.RAPOD_FAR);
                    shooting = false;
                    far = false;

                }
                break;
            case SpeedMid:
                intake.setPower(0);
                linkage.setPosition(TeamConstants.LINKAGE_SHOOT);
                shooter.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
                blocker.setPosition(TeamConstants.BLOCKER_OPEN);
                transfer.setPower(0);
                if(gamepad2.right_bumper||next){
                    setState(IntakeState.RAPOD_CLOSE);
                    shooting = false;
                    close = false;
                }
                break;


            case RAPOD_CLOSE:
                shooter.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
                if(shooter.getRPM()>TeamConstants.SHOOTER_MID_RPM-100){
                    happend = true;
                }
                if (happend){
                    intake.setPower(TeamConstants.INTAKE_IN_POWER);
                    transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
                }
                break;
            case RAPOD_FAR:
                shooter.spinToRpm(TeamConstants.SHOOTER_FARAUTO_RPM);
                linkage.setPosition(TeamConstants.LINKAGE_REST);
                if(shooter.getRPM()>TeamConstants.SHOOTER_FARAUTO_RPM-50){
                    happend = true;
                }
                if(happend){
                    intake.setPower(TeamConstants.INTAKE_IN_POWER-0.3);
                    transfer.setPower(TeamConstants.TRANSFER_IN_POWER-0.3 );
                }
                break;

            case REST:
                happend = false;
                intake.setPower(TeamConstants.INTAKE_DEFAULT_POWER);
                shooter.stop();
                blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
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

        telemetry.addData("Rpm",shooter.getRPM());
    }
    public void init(){
        shooter.linkage.setPosition(TeamConstants.LINKAGE_SHOOT);
        blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
        intake.setPower(0);
        transfer.setPower(0);
        shooter.stop();
        setState(IntakeState.REST);

    }



}
