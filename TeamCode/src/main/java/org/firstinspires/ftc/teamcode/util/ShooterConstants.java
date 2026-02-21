package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.opMode.teleOp.RedTeleop;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
//inspired by Gyrobotic Droids

public class ShooterConstants extends RedTeleop {

    private Outtake outtake;
    private Intake intake;


    public static Pose GOAL_POS_RED = new Pose(138,138);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();
    public static double SCORE_HEIGHT = 38; //inches
    public static double SCORE_ANGLE = Math.toRadians(45);
    public static double PASS_THROUGH_THIS_PROB_RADIUS = 5;


    DcMotorEx motor = (DcMotorEx)hardwareMap.get(DcMotor.class, "motor");
    double velocity = motor.getVelocity(); // Ticks per seconds

    public static double getFlywheelTicksFromVelocity(double velocity){

        //define varibles after and equation for hood
return 67;
        //return MathFunctions.clamp(94.501 * velocity/12 -187.96 + flywheeloffset, FLYWHEEL_MIN_SPEED, FLYWHEEL MAX SPEED);
    }



    public static double getHoodTicksFromDegrees(double degrees){
        return 0.0226 * degrees - 0.7443;
        //insert formula above
    }
}
