package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private double turretD = 0.5;

    private HardwareMap hw;
    private Servo turretServo;
    private Servo turretServo2;
    Follower follower;

    public Turret(HardwareMap hw, Follower follower){
        this.hw = hw;
        this.follower = follower;
        turretServo = hw.get(Servo.class, "TurretServo");
        turretServo2 = hw.get(Servo.class, "TurretServo2");
        turretServo2.setDirection(Servo.Direction.REVERSE);
    }

    public void update(Gamepad gamepad, Gamepad gamepad1){
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
        if(degreeDiff<120&&degreeDiff>90){
            degreeDiff = 90;
        }else if(degreeDiff >360-120&&degreeDiff<360-90){
        degreeDiff = -90;
        }
        else if(degreeDiff>360-90){
            degreeDiff = 360-degreeDiff;
            degreeDiff *=-1;
        }
        else if(degreeDiff>120&&degreeDiff<360-120){
            move = false;
        }
        double servoPos = ((0.00555555555) * degreeDiff) + 0.5;
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

}
