package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.lights.RGBIndicator;
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
import org.firstinspires.ftc.teamcode.util.ShootWhileMoving;
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

    ElapsedTime timer = new ElapsedTime();

    public DcMotorEx transfer;

    private final Outtake shooter;
    private final Telemetry telemetry;
    private boolean shooting = false;

    InterpLUT interpLUT;
    InterpLUT interpLUTshoot;


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
        interpLUT.add(0, 2500);
        interpLUT.add(24, 2900);
        interpLUT.add(39, 3000);
        interpLUT.add(63, 3200);
        interpLUT.add(78, 3420);
        interpLUT.add(86, 3550);
        interpLUT.add(100, 3550);

        interpLUT.add(120, 3700);

        interpLUT.add(132, 4400);
        interpLUT.add(170, 4600);
        interpLUT.add(5000, 4600);

        interpLUTshoot = new InterpLUT();
        interpLUTshoot.add(0, 0);
        interpLUTshoot.add(24, 0);
        interpLUTshoot.add(39, 0.5);
        interpLUTshoot.add(63, 0.75);
        interpLUTshoot.add(78, 0.75);
        interpLUTshoot.add(86, 0.8);
        interpLUTshoot.add(120, 0.9);

        interpLUTshoot.add(132, 1);
        interpLUTshoot.add(170, 1);
        interpLUTshoot.add(5000, 1);





        interpLUT.createLUT();
        interpLUTshoot.createLUT();

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
    public Intake(HardwareMap hw, Telemetry t, Outtake shooter, Follower follower, boolean red) {
        this.telemetry = t;
        this.shooter = shooter;
        this.follower = follower;
        next = false;
        far = false;
        close= false;
        interpLUT = new InterpLUT();
        interpLUT.add(0, 2500);
        interpLUT.add(24, 2900);
        interpLUT.add(39, 3000);
        interpLUT.add(63, 3200);
        interpLUT.add(78, 3420);
        interpLUT.add(86, 3550);
        interpLUT.add(100, 3550);

        interpLUT.add(120, 3700);

        interpLUT.add(132, 4400);
        interpLUT.add(170, 4600);
        interpLUT.add(5000, 4600);

        interpLUTshoot = new InterpLUT();
        interpLUTshoot.add(0, 0);
        interpLUTshoot.add(24, 0);
        interpLUTshoot.add(39, 0.5);
        interpLUTshoot.add(63, 0.75);
        interpLUTshoot.add(78, 0.75);
        interpLUTshoot.add(86, 0.8);
        interpLUTshoot.add(120, 0.9);

        interpLUTshoot.add(132, 1);
        interpLUTshoot.add(170, 1);
        interpLUTshoot.add(5000, 1);





        interpLUT.createLUT();
        interpLUTshoot.createLUT();

        // Map hardware
        intake = hw.get(DcMotor.class, "Intake");
        linkage = hw.get(Servo.class, "Linkage");
        transfer = hw.get(DcMotorEx.class, "Transfer");
        blocker = hw.get(Servo.class, "blocker");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        blocker.setPosition(0.5);
        if(red){
            goal = new Pose(133,133,0);
        }else{
            goal = new Pose(11,133,0);
        }

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


        long t = sm.timeInStateMs();

        // -----------------------------
        // STATE LOGIC
        // -----------------------------
        switch (sm.getState()) {

            case IntakeNEXT:
                shooter.setPower(TeamConstants.SHOOTER_CLOSED);
//                blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
                intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
                transfer.setPower(TeamConstants.TRANSFER_INTAKE_POWER);
                break;

            case AUTORPMRED:
                double distance = 0.0;
                if(false){
                   Pose tempGaol; goal = ShootWhileMoving.getCompensatedGoal(follower,goal);
                   distance = follower.getPose().distanceFrom(goal);
                }else{
                    distance = follower.getPose().distanceFrom(goal);

                }
                shooter.linkage.setPosition(interpLUTshoot.get(distance));

                double rpm = interpLUT.get(distance);
                shooter.spinToRpm(rpm);
                if(shooter.getRPM()>=rpm&&shooter.getRPM()<rpm +50){
                    happend = true;

                }
                if (happend){
                    intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
                    transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
                    blocker.setPosition(0.58);
                }else{
                    intake.setPower(0);
                    transfer.setPower(0);
                }
                break;
            case RUNSLOW:
                intake.setPower(TeamConstants.INTAKE_REVERSED);
                transfer.setPower(TeamConstants.TRANSFER_REVERSED);
                shooter.setPower(TeamConstants.SLIGHT_REVERSE_OUTTAKE);

                break;

            case SpeedFar:
                //kp is 0.07
                //kd is 0.00001
                intake.setPower(TeamConstants.INTAKE_STOP);
                linkage.setPosition(TeamConstants.LINKAGE_SHOOT);
                shooter.spinToRpm(TeamConstants.SHOOTER_FARAUTO_RPM);
//                blocker.setPosition(TeamConstants.BLOCKER_OPEN);
                transfer.setPower(TeamConstants.TRANSFER_CLOSED);
                if(gamepad2.right_bumper||next){
                    setState(IntakeState.RAPOD_FAR);
                    shooting = false;
                    far = false;

                }
                break;
            case SpeedMid:
                intake.setPower(TeamConstants.INTAKE_STOP);
                linkage.setPosition(TeamConstants.LINKAGE_SHOOT);
                shooter.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
//                blocker.setPosition(TeamConstants.BLOCKER_OPEN);
                transfer.setPower(TeamConstants.TRANSFER_CLOSED);
                if(gamepad2.right_bumper||next){
                    setState(IntakeState.RAPOD_CLOSE);
                    shooting = false;
                    close = false;
                }
                break;


            case RAPOD_CLOSE:
                shooter.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
                if(shooter.getRPM()>TeamConstants.SHOOTER_MID_RPM-30){
                    happend = true;
                }
                if (happend){
                    intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
                    transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
                }
                break;
            case RAPOD_FAR:
                shooter.spinToRpm(TeamConstants.SHOOTER_FARAUTO_RPM);
                linkage.setPosition(TeamConstants.LINKAGE_SHOOT);
                if(shooter.getRPM()>TeamConstants.SHOOTER_FARAUTO_RPM-30){
                    happend = true;
                }
                if(happend){
                    intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
                    transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
                }
                break;

            case REST:
                happend = false;
                intake.setPower(TeamConstants.INTAKE_STOP);
                shooter.stop();
//                blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
                transfer.setPower(TeamConstants.TRANSFER_CLOSED);
                linkage.setPosition(TeamConstants.LINKAGE_REST);
                break;
            default:
                happend = false;
//                blocker.setPosition(TeamConstants.BLOCKER_CLOSE);

                intake.setPower(TeamConstants.INTAKE_STOP);
                shooter.stop();
                transfer.setPower(TeamConstants.TRANSFER_CLOSED);
                linkage.setPosition(TeamConstants.LINKAGE_REST);
                break;
        }
        telemetry.addData("Intake state", sm.getState());
        telemetry.addData("RPM",shooter.getRPM());
    }
    public void init(){
        shooter.linkage.setPosition(TeamConstants.LINKAGE_SHOOT);
//        blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
        intake.setPower(0);
        transfer.setPower(0);
        shooter.stop();
        setState(IntakeState.REST);

    }



}
