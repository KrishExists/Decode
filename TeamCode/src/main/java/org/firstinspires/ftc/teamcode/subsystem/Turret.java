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

    public Turret(HardwareMap hw, Telemetry telemetry, Follower follower){
        auto = false;
        this.hw = hw;
        this.follower = follower;
        turretServo = hw.get(Servo.class, "TurretServo");
        turretServo2 = hw.get(Servo.class, "TurretServo2");
        turretServo2.setDirection(Servo.Direction.REVERSE);
        automove = false;
    }
    private void auto(){
        //Step 1 get locked heading
        double followerx = follower.getPose().getX();
        double followery = follower.getPose().getY();

        Pose Goal = new Pose(133,133);

        double goalx = Goal.getX();
        double goaly = Goal.getY();

        double lockedHeading = Math.atan2(goaly - followery, goalx - followerx);;

        double robotheading = follower.getHeading();
        double difference = robotheading - lockedHeading;

        double degreeDiff = Math.toDegrees(difference);
        boolean move = true;
        // Step 2
        //aroudn to is 90,360-90
        if(degreeDiff<=120&&degreeDiff>90){
            degreeDiff = 90;
        }else if(degreeDiff >360-120&&degreeDiff<360-90){
            degreeDiff = -90;
        }
        else if(degreeDiff>=360-90){
            degreeDiff = 360-degreeDiff;
            degreeDiff *=-1;
        }
        else if(degreeDiff>120&&degreeDiff<360-120){
            move = false;
        }double gearRatio = 35.0 / 36.0;
        double servoPos = (0.00555555555 * degreeDiff * gearRatio) + turretOriginal;
        if(servoPos>1){
            servoPos = 1;
        } else if (servoPos<0) {
            servoPos = 0;
        }
        if(move) {

            turretServo.setPosition(servoPos);
            turretServo2.setPosition(servoPos);
        }
    }
    private void autoMove(){
        //Step 1 get locked heading
        double followerx = follower.getPose().getX();
        double followery = follower.getPose().getY();

        Pose Goal = ShootWhileMoving.getCompensatedGoal(follower,new Pose(133,133));
        double goalx = Goal.getX();
        double goaly = Goal.getY();

        double lockedHeading = Math.atan2(goaly - followery, goalx - followerx);;

        double robotheading = follower.getHeading();
        double difference = robotheading - lockedHeading;

        double degreeDiff = Math.toDegrees(difference);
        boolean move = true;
        // Step 2
        //aroudn to is 90,360-90
        if(degreeDiff<=120&&degreeDiff>90){
            degreeDiff = 90;
        }else if(degreeDiff >360-120&&degreeDiff<360-90){
            degreeDiff = -90;
        }
        else if(degreeDiff>=360-90){
            degreeDiff = 360-degreeDiff;
            degreeDiff *=-1;
        }
        else if(degreeDiff>120&&degreeDiff<360-120){
            move = false;
        }double gearRatio = 35.0 / 36.0;
        double servoPos = (0.00555555555 * degreeDiff * gearRatio) + turretOriginal;
        if(servoPos>1){
            servoPos = 1;
        } else if (servoPos<0) {
            servoPos = 0;
        }
        if(move) {

            turretServo.setPosition(servoPos);
            turretServo2.setPosition(servoPos);
        }
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
               auto();

           }else{
               autoMove();
           }
       }else{
           manual();
       }
    }

}
