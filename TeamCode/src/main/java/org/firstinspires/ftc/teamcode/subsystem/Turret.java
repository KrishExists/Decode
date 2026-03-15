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
    public Servo turretServo;
    public static double SLOPE = 0.00372222;
    public Servo turretServo2;
    Follower follower;
    private boolean auto;
    public static boolean automove;
    Pose goal1;
    Telemetry telemetry;
    private boolean red;

    public Turret(HardwareMap hw, Telemetry telemetry, Follower follower){
        auto = false;
        this.hw = hw;
        this.follower = follower;
        this.telemetry = telemetry;
        turretServo = hw.get(Servo.class, "TurretServo");
        turretServo2 = hw.get(Servo.class, "TurretServo2");

        automove = false;
        goal1 = new Pose(144,144);
    }
    public Turret(HardwareMap hw, Telemetry telemetry, Follower follower, boolean red){
        auto = false;
        this.hw = hw;
        this.follower = follower;
        this.telemetry = telemetry;
        turretServo = hw.get(Servo.class, "TurretServo");
        turretServo2 = hw.get(Servo.class, "TurretServo2");
        turretServo.setPosition(0.5);
        turretServo2.setPosition(0.5);

        automove = false;
        this.red = red;
        if(red){
            goal1 = new Pose(144,144);
        }else{
            goal1 = new Pose(0,144);
        }
    }

    public void auto(Pose Goal){
        telemetry.addLine("--------------Turret Tele Data ------------");
        telemetry.addLine("red turret");
        //Step 1 get locked heading
        telemetry.addData("Red = ", red);
        double followerx = follower.getPose().getX();
        double followery = follower.getPose().getY();
        double goalx = Goal.getX();
        double goaly = Goal.getY();


        telemetry.addData("Goal Pose:", "(" + goalx + "," + goaly + ")");

        double lockedHeading = Math.atan2(goaly - followery, goalx - followerx);
        telemetry.addData("lockedHeading",Math.toDegrees(lockedHeading));

        double robotheading = follower.getHeading();
        telemetry.addData("robot heading",robotheading);
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
        double servoPos = SLOPE * degreeDiff + 0.5;
        telemetry.addData("Servo pos no clamp",servoPos);
        if(servoPos>0.9){
            servoPos = 0.9;
        } else if (servoPos<0.1) {
            servoPos = 0.1;
        }
        telemetry.addData("servopos clamp",servoPos);
        if(move) {
            telemetry.addLine("doing servo pose");
            turretServo.setPosition(servoPos);
            turretServo2.setPosition(servoPos);
        }
    }
//    public void blue(Pose Goal){
//        //Step 1 get locked heading
//        double followerx = follower.getPose().getX();
//        double followery = follower.getPose().getY();
//        double goalx = Goal.getX();
//        double goaly = Goal.getY();
//
//        double lockedHeading = Math.atan2(goaly - followery, goalx - followerx);
//        telemetry.addData("lockedHeading",lockedHeading);
//
//        double robotheading = 90+(90-follower.getHeading());
//        double difference = robotheading - lockedHeading;
//        double degreeDiff = Math.toDegrees(difference) * -1;
//        telemetry.addData("degree diff",degreeDiff);
//        boolean move = true;
//        // Step 2
//        //aroudn to is 90,360-90
//        if(degreeDiff<=120&&degreeDiff>90){//turret hysteria right 90 is max here so thats why
//            degreeDiff = 90;
//            move = true;
//
//        }else if(degreeDiff >360-120&&degreeDiff<360-90){//turret hysteria 120 is max so 30 range
//            degreeDiff = -90;
//            move = true;
//
//        }
//        else if(degreeDiff>=360-90){//helps do the negative sign instead of it being really large
//            degreeDiff = 360-degreeDiff;
//            degreeDiff *=-1;
//            move = true;
//
//        }
//        else if(degreeDiff>120&&degreeDiff<360-120){//anything diff here
//            move = false;
//        }
//        telemetry.addData("Degree diff 2",degreeDiff);
//        telemetry.addData("move",move);
//        double servoPos = SLOPE * degreeDiff + 0.5;
//        telemetry.addData("Servo pos no clamp",servoPos);
//        if(servoPos>0.9){
//            servoPos = 0.9;
//        } else if (servoPos<0.1) {
//            servoPos = 0.1;
//        }
//        telemetry.addData("servopos clamp",servoPos);
//        if(move) {
//            telemetry.addLine("doing servo pose");
//            turretServo.setPosition(servoPos);
//            turretServo2.setPosition(servoPos);
//        }
//    }
public void auto2(Pose Goal){
    red = false;
    telemetry.addLine("--------------Turret Tele Data ------------");
    telemetry.addLine("blue turret");
    telemetry.addData("Red = ", red);

    double followerx = follower.getPose().getX();
    double followery = follower.getPose().getY();
    double goalx = Goal.getX();
    double goaly = Goal.getY();

    telemetry.addData("Goal Pose:", "(" + goalx + "," + goaly + ")");

    double lockedHeading = Math.atan2(goaly - followery, goalx - followerx);
    telemetry.addData("lockedHeading", Math.toDegrees(lockedHeading));

    // Mirror blue side by offsetting robot heading by 180 degrees
    double robotheading = follower.getHeading() + 2*Math.PI;
    if (robotheading >= 2 * Math.PI) robotheading -= 2 * Math.PI;
    telemetry.addData("robot heading mirrored degrees", Math.toDegrees(robotheading));

    double difference = robotheading - lockedHeading;
    double degreeDiff = Math.toDegrees(difference);
    telemetry.addData("degree diff", degreeDiff);

    boolean move = true;

    // Exact same hysteria logic as auto()
    if (degreeDiff <= 120 && degreeDiff > 90) {
        degreeDiff = 90;
        move = true;
    } else if (degreeDiff > 360-120 && degreeDiff < 360-90) {
        degreeDiff = -90;
        move = true;
    } else if (degreeDiff >= 360-90) {
        degreeDiff = 360 - degreeDiff;
        degreeDiff *= -1;
        move = true;
    } else if (degreeDiff > 120 && degreeDiff < 360-120) {
        move = false;
    }

    telemetry.addData("Degree diff 2", degreeDiff);
    telemetry.addData("move", move);

    double servoPos = SLOPE * degreeDiff + 0.5;
    telemetry.addData("Servo pos no clamp", servoPos);

    if (servoPos > 0.9) {
        servoPos = 0.9;
    } else if (servoPos < 0.1) {
        servoPos = 0.1;
    }

    telemetry.addData("servopos clamp", servoPos);

    if (move) {
        telemetry.addLine("doing servo pose");
        turretServo.setPosition(servoPos);
        turretServo2.setPosition(servoPos);
    } else {
        telemetry.addLine("goal out of range - holding last position");
    }
}    public void autotelem(){
        //Step 1 get locked heading for turret
        double followerx = follower.getPose().getX();
        double followery = follower.getPose().getY();


        double goalx = goal1.getX();
        double goaly = goal1.getY();

        double lockedHeading = Math.atan2(goaly - followery, goalx - followerx);
        telemetry.addData("lockedHeading",lockedHeading);

        double robotheading = follower.getHeading();
        telemetry.addData("robot heading", Math.toDegrees(robotheading));
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
        degreeDiff *=-1;
        telemetry.addData("Degree diff 2",degreeDiff);
        telemetry.addData("move",move);
        double servoPos = 0.00444444 * degreeDiff + 0.5;//stuff
        telemetry.addData("Servo pos no clamp",servoPos);
        if(servoPos>0.85){
            servoPos = 0.9;
        } else if (servoPos<0.15) {
            servoPos = 0.1;
        }
        telemetry.addData("servopos clamp",servoPos);
        if(true) {
            telemetry.addLine("doing servo pose");
            telemetry.addData("serov pos in loop",servoPos);
            turretServo.setPosition(servoPos);
            turretServo2.setPosition(servoPos);
        }
    }


    public void autoMove(){
        //Step 1 get locked heading

        Pose Goal = ShootWhileMoving.getCompensatedGoal(follower,new Pose(144,144));
        auto(Goal);

    }
    public void manual(){
        turretServo.setPosition(turretOriginal);
        turretServo2.setPosition(turretOriginal);
    }

    public void setPosition(double position){
        turretServo.setPosition(position);
        turretServo2.setPosition(position);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2){
       if(gamepad2.leftBumperWasPressed()){
           auto = !auto;
       }
       if(auto){
           if(red) {
               auto(new Pose(144, 144));
           }
           if(!red){
               auto2(new Pose(0, 144));}
               telemetry.addLine("Auto");
//           }else{
//               autoMove();
//               telemetry.ad
//
//               dLine("Automove");
//
//           }
       }else{
           manual();
           telemetry.addLine("manual");

       }
    }

    public void blue(Pose goal) {

    }

    public void updatetelem(){
        autotelem();
        telemetry.addData("Turret pose", follower.getPose());
    }





}
