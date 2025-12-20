package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

import java.util.Vector;

public class OdometryAutoALign implements Subsystem{

    private DcMotor lfm, lbm, rfm, rbm;

    private MecanumDrive follower;

    private static double GOAL_X = -60;
    private static double GOAL_Y = -60;

    private Vector2d robotPose = new Vector2d(0,0);

    private double robotHeading = 0;

    public boolean isALigning = false;

    Telemetry telemetry;

    public OdometryAutoALign(HardwareMap map, Telemetry telemetry){
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lfm = map.get(DcMotor.class, "frontLeft");
        lbm = map.get(DcMotor.class, "backLeft");
        rfm = map.get(DcMotor.class, "frontRight");
        rbm = map.get(DcMotor.class, "backRIght");
    }


    @Override
    public void init() {
        telemetry.addData("Turret", "Initialized");
        telemetry.update();

    }

    public void updatePose(Vector2d pos, double headingDeg){
        this.robotPose = pos;
        this.robotHeading = headingDeg;
    }

    private double calculateDistanceToGoal(Vector2d robotPose, Vector2d goalPos){
        Vector2d toGoal = goalPos.minus(robotPose);
        return Math.sqrt(toGoal.x * toGoal.x + toGoal.y * toGoal.y);
    }

    private  double calculateRobotAngle(Vector2d robotPose, double robotHeading, Vector2d goalPose){
        Vector2d toGoal = goalPose.minus(robotPose);
        double angleToGoal = Math.toDegrees(Math.atan2(toGoal.y, toGoal.x));
        angleToGoal = AngleUnit.normalizeDegrees(angleToGoal);
        double robotAngle = AngleUnit.normalizeDegrees(angleToGoal-robotHeading);
        return robotAngle;
    }

    private void setRobotPosition(double robotAngleDeg){
        double robotPositionn = 0;
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2) {


    }
}
