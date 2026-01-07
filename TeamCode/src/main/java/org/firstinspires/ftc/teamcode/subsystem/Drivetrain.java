package org.firstinspires.ftc.teamcode.subsystem;
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


public class Drivetrain implements Subsystem {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;


    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private static boolean teleDrive = true;



    public Drivetrain(HardwareMap h, Telemetry t) {
        startingPose = new Pose(72,72,0);//middle
        follower = Constants.createFollower(h);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(72, 22))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(55), 0.8))
                .build();

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
            follower.followPath(pathChain.get());
        } else if (automatedDrive && (gamepad.bWasPressed() || !follower.isBusy())) {
            automatedDrive = false;
            follower.startTeleopDrive(true); // ensure teleop drive restarts
        }

        // Drive based on mode
        if (automatedDrive) {
            telemetry.addData("Mode", "Automated Path Following");
        } else {
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

