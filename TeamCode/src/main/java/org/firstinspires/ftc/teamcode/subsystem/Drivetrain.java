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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Configurable
public class Drivetrain implements Subsystem {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> far;
    private Supplier<PathChain> mid;
    private Supplier<PathChain> park;

    private Supplier<PathChain> gate;

    private TelemetryManager telemetryM;


    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private static boolean teleDrive = true;



    public Drivetrain(HardwareMap h, Telemetry t) {
        this(h,t,false);
    }
    public Drivetrain(HardwareMap h, Telemetry t, boolean red) {
        startingPose = new Pose(72,72,0);//middle
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
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(38, 33))))
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
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(75, 75))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
                    .build();
            park = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(38, 33))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                    .build();
            gate = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(130, 69))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, follower::getHeading, 0.8))
                    .build();
        }

        this.hardwareMap = h;
        this.telemetry = t;
        follower.startTeleopDrive(teleDrive);
    }



    // ----------------------------------------
    // MANUAL DRIVE (converted from your old Drive.java)
    // ----------------------------------------
//    public void manualDrive(Gamepad gamepad1) {
//        double y = -gamepad1.left_stick_y;
//        double x = gamepad1.left_stick_x * 1.1;
//        double rx = gamepad1.right_stick_x;
//
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
//
//        drive.leftFront.setPower((y + x + rx) / denominator);
//        drive.leftBack.setPower((y - x + rx) / denominator);
//        drive.rightFront.setPower((y - x - rx) / denominator);
//        drive.rightBack.setPower((y + x - rx) / denominator);
//    }

    public void combinedDrive(Gamepad gamepad) {

        // Check for path triggers first
        if (gamepad.aWasPressed() && !automatedDrive) {
            automatedDrive = true;
            Intake.far = true;

            follower.followPath(far.get());
        }
      else  if (gamepad.xWasPressed() && !automatedDrive) {
            Intake.close = true;

            automatedDrive = true;
            follower.followPath(mid.get());
        }
      else    if (gamepad.yWasPressed() && !automatedDrive) {
            automatedDrive = true;
            follower.followPath(park.get());
        }
        else    if (gamepad.rightBumperWasPressed() && !automatedDrive) {
            automatedDrive = true;
            follower.followPath(gate.get());
        }
        else if (automatedDrive && (gamepad.bWasPressed())) {
            automatedDrive = false;
            follower.startTeleopDrive(true); // ensure teleop drive restarts
        }
        if(automatedDrive&&!follower.isBusy()){
            Intake.next = true;

        }

        // Drive based on mode
        if (automatedDrive) {
            telemetry.addData("Mode", "Automated Path Following");
        } else {
            Intake.far = false;
            Intake.close = false;
            Intake.next = false;

            // Robot-centric teleop drive
            follower.setTeleOpDrive(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x,
                    -gamepad.right_stick_x,
                    true
            );
            telemetry.addData("Mode", "Manual Teleop");
        }

        // Always add common telemetry
        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.addData("AutomatedDrive", automatedDrive);
    }



    @Override
    public void init() {}

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

