

package org.firstinspires.ftc.teamcode.opMode.teleOp.testers;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.function.Supplier;


@TeleOp(name = "RedTeleop", group = "Main")
public class pedroLocalizatoinTest extends LinearOpMode {
    private HardwareMap hw;
    private driveTests drive;

    @Override
    public void runOpMode() throws InterruptedException {


// ===== Initialize Hardware & Subsystems =====
        hw = hardwareMap;
        drive = new driveTests(hardwareMap, telemetry);
        telemetry.addLine("Initialized — Waiting for Start");
        telemetry.update();
        Drawing.init();

        waitForStart();
        if (isStopRequested()) return;


// ===== Main Loop =====
        while (opModeIsActive()) {
            drive.update(gamepad1);

            telemetry.update();



        }
    }


}

class driveTests implements Subsystem {
    private final Supplier<PathChain> start;
    private static Follower follower;
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

    private final TelemetryManager telemetryM;


    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private static final boolean teleDrive = true;

    public driveTests(HardwareMap h, Telemetry t) {

        startingPose = new Pose(72,72,0);
        drawOnlyCurrent();
        follower = Constants.createFollower(h);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
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
    public static void draw() {
        Drawing.drawDebug(follower);
    }
    public static void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
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
        draw();
        telemetry.addData("position", follower.getPose());
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("automatedDrive", automatedDrive);
    }
}

class Drawing {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}

