package org.firstinspires.ftc.teamcode.subsystem;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import java.util.function.Supplier;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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

    private final Supplier<PathChain> gate;
    private boolean startFlag = false;
    private boolean parkFlag = false;
    private boolean gateFlag = false;
    private boolean closeFlag = false; // your "mid/close"
    private boolean farFlag = false;

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    private final TelemetryManager telemetryM;


    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private static final boolean teleDrive = true;


    public Drivetrain(HardwareMap h, Telemetry t) {
        this(h,t,false);
    }
    public Drivetrain(HardwareMap h, Telemetry t, boolean red) {
        if(PoseStorage.pose!=null){
            startingPose = PoseStorage.pose;
            t.addData("Got pose",startingPose);
        }else{
        }
        startingPose = new Pose(72,72,0);//to fix subtract 10 from x and add 10 to y. however this should not be thte case. for some reason its misinterpeting the pose by 10

        follower = Constants.createFollower(h);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        if(red){
            far = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(72, 22))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(70), 0.8))
                    .build();
            mid = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(75, 75))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                    .build();
            park = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(32, 37))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                    .build();
            gate = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(130, 69))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, follower::getHeading, 0.8))
                    .build();
        }else{
            far = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(72, 22))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(110), 0.8))
                    .build();
            mid = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(69, 75))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
                    .build();
            park = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(72-38+72, 33))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                    .build();
            gate = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(72-130+72, 69))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, follower::getHeading, 0.8))
                    .build();
        }
        start = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(72, 72))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, 0, 0.8))
                .build();

        this.hardwareMap = h;
        this.telemetry = t;
        follower.startTeleopDrive(teleDrive);
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

    public void combinedDrive(Gamepad gamepad) {

        // ====== BUTTONS SET FLAGS ======
        if (gamepad.yWasPressed()) startFlag = true;
        if (gamepad.right_trigger > 0.2) parkFlag = true;
        if (gamepad.left_trigger > 0.2) gateFlag = true;
        if (gamepad.leftBumperWasPressed()) closeFlag = true;
        if (gamepad.rightBumperWasPressed()) farFlag = true;

        // ====== PRIORITY EXECUTION (NO ELSE-IFS) ======
        if (!automatedDrive) {

            if (startFlag) {
                automatedDrive = true;
                follower.followPath(start.get());
                startFlag = false;
            }

            if (parkFlag) {
                automatedDrive = true;
                follower.followPath(park.get());
                parkFlag = false;
            }

            if (gateFlag) {
                automatedDrive = true;
                follower.followPath(gate.get());
                gateFlag = false;
            }

            if (closeFlag) {
                automatedDrive = true;
                follower.followPath(mid.get());
                closeFlag = false;
            }

            if (farFlag) {
                automatedDrive = true;
                follower.followPath(far.get());
                farFlag = false;
            }
        }

        if (automatedDrive) {
            // check if driver moved sticks beyond deadzone
            boolean joystickInput = Math.abs(gamepad.left_stick_x) > 0.1
                    || Math.abs(gamepad.left_stick_y) > 0.1
                    || Math.abs(gamepad.right_stick_x) > 0.1;

            // or pressed B to cancel
            if (joystickInput || gamepad.bWasPressed()) {
                automatedDrive = false;
                follower.startTeleopDrive(true); // ensure teleop drive restarts
            }
        }

// ====== MANUAL DRIVE ======
        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x,
                    -gamepad.right_stick_x,
                    true
            );
        }

        telemetry.addData("Mode", automatedDrive ? "Auto" : "Manual");
        telemetry.addData("Gamepad leftX", gamepad.left_stick_x);
        telemetry.addData("Gamepad lefty", gamepad.left_stick_y);
        telemetry.addData("Gamepad rX", gamepad.right_stick_x);

        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.addData("AutomatedDrive", automatedDrive);
    }



    @Override
    public void init() {
        rightBack = hardwareMap.get(DcMotor.class, "backRightMotor");
        rightFront = hardwareMap.get(DcMotor.class, "frontRightMotor");
        leftBack = hardwareMap.get(DcMotor.class, "backLeftMotor");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        update(gamepad1);
    }

    public void update(Gamepad gamepad) {
        follower.update();

        this.combinedDrive(gamepad);
        telemetry.addData("position", follower.getPose());
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("automatedDrive", automatedDrive);
    }
}

