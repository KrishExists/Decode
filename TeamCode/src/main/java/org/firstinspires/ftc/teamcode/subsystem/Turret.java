package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ShootWhileMoving;

@Config
public class Turret implements Subsystem {
    private double turretOriginal = 0.5;

    private HardwareMap hw;
    private Servo turretServo;
    private Servo turretServo2;
    Follower follower;
    private boolean auto;
    public static boolean automove;
    Pose goal1;
    Telemetry telemetry;

    public Turret(HardwareMap hw, Telemetry telemetry, Follower follower){
        auto = false;
        this.hw = hw;
        this.follower = follower;
        this.telemetry = telemetry;
        turretServo = hw.get(Servo.class, "TurretServo");
        turretServo2 = hw.get(Servo.class, "TurretServo2");
        turretServo2.setDirection(Servo.Direction.REVERSE);
        automove = false;
        goal1 = new Pose(133,133);
    }
    private void auto(Pose Goal){
        //Step 1 get locked heading
        double followerx = follower.getPose().getX();
        double followery = follower.getPose().getY();
        double goalx = Goal.getX();
        double goaly = Goal.getY();

        double lockedHeading = Math.atan2(goaly - followery, goalx - followerx);;
        telemetry.addData("lockedHeading",lockedHeading);

        double robotheading = follower.getHeading();
        double difference = robotheading - lockedHeading;
        double degreeDiff = Math.toDegrees(difference);
        telemetry.addData("degree diff",degreeDiff);
        boolean move = true;
        // Step 2
        //aroudn to is 90,360-90
        if(degreeDiff<=120&&degreeDiff>90){//turret hysteria right 90 is max here so thats why
            degreeDiff = 90;
            move = true;

        }else if(degreeDiff >360-120&&degreeDiff<360-90){//turret hysteria 120 is max so 30 range
            degreeDiff = -90;
            move = true;

        }
        else if(degreeDiff>=360-90){//helps do the negative sign instead of it being really large
            degreeDiff = 360-degreeDiff;
            degreeDiff *=-1;
            move = true;

        }
        else if(degreeDiff>120&&degreeDiff<360-120){//anything diff here
            move = false;
        }
        telemetry.addData("Degree diff 2",degreeDiff);
        telemetry.addData("move",move);
        double servoPos = 0.00444444 * degreeDiff + 0.5;
        telemetry.addData("Servo pos no clamp",servoPos);
        if(servoPos>0.9){
            servoPos = 0.9;
        } else if (servoPos<0.1) {
            servoPos = 0.1;
        }
        telemetry.addData("servopos clamp",servoPos);
        if(move) {

            turretServo.setPosition(servoPos);
            turretServo2.setPosition(servoPos);
        }
    }
    private void autotelem(){
        //Step 1 get locked heading
        double followerx = follower.getPose().getX();
        double followery = follower.getPose().getY();


        double goalx = goal1.getX();
        double goaly = goal1.getY();

        double lockedHeading = Math.atan2(goaly - followery, goalx - followerx);;
        telemetry.addData("lockedHeading",lockedHeading);

        double robotheading = follower.getHeading();
        double difference = robotheading - lockedHeading;
        double degreeDiff = Math.toDegrees(difference);
        telemetry.addData("degree diff",degreeDiff);
        boolean move = true;
        // Step 2
        //aroudn to is 90,360-90
        if(degreeDiff<=120&&degreeDiff>90){//turret hysteria right 90 is max here so thats why
            degreeDiff = 90;
            move = true;

        }else if(degreeDiff >360-120&&degreeDiff<360-90){//turret hysteria 120 is max so 30 range
            degreeDiff = -90;
            move = true;

        }
        else if(degreeDiff>=360-90){//helps do the negative sign instead of it being really large
            degreeDiff = 360-degreeDiff;
            degreeDiff *=-1;
            move = true;

        }
        else if(degreeDiff>120&&degreeDiff<360-120){//anything diff here
            move = false;
        }
        telemetry.addData("Degree diff 2",degreeDiff);
        telemetry.addData("move",move);
        double servoPos = 0.00444444 * degreeDiff + 0.5;
        telemetry.addData("Servo pos no clamp",servoPos);
        if(servoPos>0.9){
            servoPos = 0.9;
        } else if (servoPos<0.1) {
            servoPos = 0.1;
        }
        telemetry.addData("servopos clamp",servoPos);
        if(move) {

            turretServo.setPosition(servoPos);
            turretServo2.setPosition(servoPos);
        }
    }
    private void autoMove(){
        //Step 1 get locked heading

        Pose Goal = ShootWhileMoving.getCompensatedGoal(follower,new Pose(133,133));
        auto(Goal);

    }
    private void manual(){
        turretServo.setPosition(turretOriginal);
        turretServo2.setPosition(turretOriginal);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2){
       if(gamepad2.leftBumperWasPressed()){
           auto = !auto;
       }
       if(auto){
           if(!automove){
               auto(goal1);
            telemetry.addLine("Auto");

           }else{
               autoMove();
               telemetry.addLine("Automove");

           }
       }else{
           manual();
           telemetry.addLine("manual");

       }
    }

    public void updatetelem(){
        autotelem();
        telemetry.addData("Turret pose", follower.getPose());
    }

}
