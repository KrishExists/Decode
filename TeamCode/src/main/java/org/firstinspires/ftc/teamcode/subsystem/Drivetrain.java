package org.firstinspires.ftc.teamcode.subsystem;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import java.util.function.Supplier;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Arc;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Config
@Configurable
public class Drivetrain implements Subsystem {
    private final Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this

    private static double kp = 3.0;


    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    private final TelemetryManager telemetryM;


    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private static final boolean teleDrive = true;
    Pose redThing;
    private double servoStart = 0.5;
    private Pose reset;
    PController headingController;
    private char[] motifpattern;
    private int index = 0;


    public Drivetrain(HardwareMap h, Telemetry t) {
        this(h,t,false);
    }
    public Drivetrain(HardwareMap h, Telemetry t, boolean red) {
        if(PoseStorage.pose!=null){
            //startingPose = PoseStorage.pose;
            startingPose = PoseStorage.pose;
            t.addData("Got pose",startingPose);
        }
        else { // I t is
            startingPose = new Pose(72, 72, 0);//to fix subtract 10 from x and add 10 to y. however this should not be thte case. for some reason its misinterpeting the pose by 10
        }

        follower = Constants.createFollower(h);
        follower.setStartingPose(startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        if(red){
            redThing = new Pose(131,136);
            reset = new Pose(9.382,8.67,Math.toRadians(180));
            //9.382  8.67   180
        }else{
            reset = new Pose(133,8.67,Math.toRadians(0));
            redThing = new Pose(0,144);
        }

        this.hardwareMap = h;
        this.telemetry = t;
        headingController = new PController(kp);
        follower.startTeleopDrive(teleDrive);
        rightBack = hardwareMap.get(DcMotor.class, "backRightMotor");
        rightFront = hardwareMap.get(DcMotor.class, "frontRightMotor");
        leftBack = hardwareMap.get(DcMotor.class, "backLeftMotor");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeftMotor");

    }



    // ----------------------------------------
    // MANUAL DRIVE (converted from your old Drive.java)
    // ----------------------------------------
    public void manualDrive(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        leftFront.setPower((y + x + rx) / denominator);
        leftBack.setPower((y - x + rx) / denominator);
        rightFront.setPower((y - x - rx) / denominator);
        rightBack.setPower((y + x - rx) / denominator);
    }
    public Follower returnFollwer(){
        return follower;
    }

    public void combinedDrive(Gamepad gamepad) {
        if(gamepad.dpad_up){
            follower.setPose(reset);
        }


            follower.setTeleOpDrive(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x,
                    -gamepad.right_stick_x,
                    true
            );

    }

    public double distanceToGoal(){
        return follower.getPose().distanceFrom(redThing);
    }

    public Pose returnPose()
    {
        return follower.getPose();
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        follower.update();
        headingController.setP(kp);
        if(index!=3){
            if(gamepad1.left_bumper){
                motifpattern[index] = 'P';
                index++;
            }if(gamepad1.right_bumper){
                motifpattern[index] = 'G';
                index++;
            }
        }
        if(index == 3){

        }
        this.combinedDrive(gamepad1);
        telemetry.addData("Pose",follower.getPose());
    }

    public void update(Gamepad gamepad) {

    }
}

