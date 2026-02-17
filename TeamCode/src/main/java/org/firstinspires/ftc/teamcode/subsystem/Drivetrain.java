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
    private final Supplier<PathChain> start;
    private final Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private final Supplier<PathChain> far;
    private final Supplier<PathChain> mid;
    private final Supplier<PathChain> park;

    private final Supplier<PathChain> human;

    private final Supplier<PathChain> gate;

    private boolean startFlag = false;
    private static double kp = 3.0;

    private boolean parkFlag = false;
    private boolean gateFlag = false;
    private boolean closeFlag = false; // your "mid/close"
    private boolean farFlag = false;
    private boolean autoFalg = false;

    private boolean humanFlag = true;

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    private final TelemetryManager telemetryM;
    private Arc myarc;


    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private static final boolean teleDrive = true;
    Pose redThing;
    private double servoStart = 0.5;
    PController headingController;


    public Drivetrain(HardwareMap h, Telemetry t) {
        this(h,t,false);
    }
    public Drivetrain(HardwareMap h, Telemetry t, boolean red) {
        if(PoseStorage.pose!=null){
            //startingPose = PoseStorage.pose;
//            startingPose = new Pose(90, 115, Math.toRadians(20));
            startingPose = PoseStorage.pose;
            t.addData("Got pose",startingPose);
        }
        else{ // I t is
            startingPose = new Pose(72,72,0);//to fix subtract 10 from x and add 10 to y. however this should not be thte case. for some reason its misinterpeting the pose by 10
        }

        follower = Constants.createFollower(h);
        follower.setStartingPose(startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        if(red){
            far = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(75, 25))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(66.16), 0.8))
                    .build();
            mid = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(85, 85))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                    .build();
            park = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(33, 38))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                    .build();

            human = () -> follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(144-106, 8))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(180), 0.8))
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(144-135, 8))))
                    .build();

            gate = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(130, 69))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, follower::getHeading, 0.8))
                    .build();
        }else{
            far = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(144-85, 18))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(180-70), 0.8))
                    .build();
            mid = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(144-85, 75))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
                    .build();
            park = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(144-32, 37))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                    .build();

            human = () -> follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(106, 8))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(180), 0.8))
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(135, 8))))
                    .build();

            gate = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(144-130, 69))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.6))
                    .build();
        }
        start = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(72, 72))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, 0, 0.8))
                .build();
        if(red){
            redThing = new Pose(144,144);

        }else{
            redThing = new Pose(0,144);
        }

        this.hardwareMap = h;
        this.telemetry = t;
        headingController = new PController(kp);
        follower.startTeleopDrive(teleDrive);
        double cx = 131;
        double cy = 135;
        double radius = Math.hypot(103 - cx, 102 - cy);
        double start = Math.PI; // due west
        double end = Math.atan2(102 - cy, 103 - cx);
        if (end < start) end += 2*Math.PI; // normalize for CCW

        myarc = new Arc(cx,cy,radius,start,end,true);


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

    public void combinedDrive(Gamepad gamepad,Gamepad gamepad2) {
        if(gamepad.dpad_up){
            follower.setPose(new Pose(72,72,0));
        }


        double follwerx = follower.getPose().getX();
        double followery = follower.getPose().getY();
        double redthingx = redThing.getX();
        double redthingy = redThing.getY();
        double angle = Math.atan2(followery-redthingy,follwerx-redthingx);
        double heading = follower.getHeading();
        heading = AngleUnit.normalizeRadians(heading);
        double error = heading-angle;

            // or pressed B to cancel

            follower.setTeleOpDrive(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x,
                    -gamepad.right_stick_x,
                    true
            );

        telemetry.addData("Mode", automatedDrive ? "Auto" : "Manual");
        telemetry.addData("Position", follower.getPose());
    }

    public double distanceToGoal(){
        return follower.getPose().distanceFrom(redThing);
    }

    public Pose returnPose()
    {
        return follower.getPose();
    }    @Override
    public void init() {
        rightBack = hardwareMap.get(DcMotor.class, "backRightMotor");
        rightFront = hardwareMap.get(DcMotor.class, "frontRightMotor");
        leftBack = hardwareMap.get(DcMotor.class, "backLeftMotor");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        follower.update();
        headingController.setP(kp);

        this.combinedDrive(gamepad1,gamepad2);
        telemetry.addData("position", follower.getPose());
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("automatedDrive", automatedDrive);
    }

    public void update(Gamepad gamepad) {

    }
}

