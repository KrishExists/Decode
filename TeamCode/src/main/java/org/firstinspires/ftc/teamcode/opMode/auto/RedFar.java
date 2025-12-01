package org.firstinspires.ftc.teamcode.opMode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Robot;

import java.lang.Math;

@Autonomous(name="RedFar", group="A")
public class RedFar extends LinearOpMode {

    // --------------------------
    // VARIABLES
    // --------------------------
    private final ElapsedTime timer = new ElapsedTime();
    Robot robot;

    // --------------------------
    // POSES FROM YOUR MEEPMEEP
    // --------------------------
    final Pose2d START = new Pose2d(60, 10, Math.toRadians(-180));

    final Pose2d P1 = new Pose2d(54, 10, Math.toRadians(-200));   // first move
    final Pose2d P2 = new Pose2d(34, 33, Math.toRadians(-270));
    final Pose2d P3 = new Pose2d(34, 45, Math.toRadians(-270));
    final Pose2d P4 = new Pose2d(54, 10, Math.toRadians(-200));
    final Pose2d P5 = new Pose2d(25, 60, 0);
    final Pose2d P6 = new Pose2d(58, 60, 0);
    final Pose2d P7 = new Pose2d(54, 10, Math.toRadians(-200));
    final Pose2d P8 = new Pose2d(28, 10, Math.toRadians(-200));

    // --------------------------
    // ACTIONS
    // --------------------------
    Action toP1, toP2, toP3, toP4, toP5, toP6, toP7, toP8;

    boolean runningAction = true;

    public enum AutoState {
        STEP_1, // Go to Shootpos
        STEP_2, // Go from p1 to spike mark
        STEP_3, // Run through spike marks
        STEP_4, // Go from spikemark to shoot
        STEP_5, // Go from shooting to triangular angular spot
        STEP_6, // Run through balls in human zone
        STEP_7, // Go to shooting pos
        STEP_8, // Leave
        END
    }

    AutoState state = AutoState.STEP_1;

    // ---------------------------------------------------------
    // BUILD ALL TRAJECTORIES
    // ---------------------------------------------------------
    public void buildPaths() {

        TrajectoryActionBuilder p1 = robot.drive.drive.actionBuilder(START)
                .strafeToLinearHeading(P1.position, P1.heading);

        TrajectoryActionBuilder p2 = robot.drive.drive.actionBuilder(P1)
                .strafeToLinearHeading(P2.position, P2.heading);

        TrajectoryActionBuilder p3 = robot.drive.drive.actionBuilder(P2)
                .strafeToLinearHeading(P3.position, P3.heading);

        TrajectoryActionBuilder p4 = robot.drive.drive.actionBuilder(P3)
                .strafeToLinearHeading(P4.position, P4.heading);

        TrajectoryActionBuilder p5 = robot.drive.drive.actionBuilder(P4)
                .strafeToLinearHeading(P5.position, P5.heading);

        TrajectoryActionBuilder p6 = robot.drive.drive.actionBuilder(P5)
                .strafeToLinearHeading(P6.position, P6.heading);

        TrajectoryActionBuilder p7 = robot.drive.drive.actionBuilder(P6)
                .strafeToLinearHeading(P7.position, P7.heading);

        TrajectoryActionBuilder p8 = robot.drive.drive.actionBuilder(P7)
                .strafeToLinearHeading(P8.position, P8.heading);

        toP1 = p1.build();
        toP2 = p2.build();
        toP3 = p3.build();
        toP4 = p4.build();
        toP5 = p5.build();
        toP6 = p6.build();
        toP7 = p7.build();
        toP8 = p8.build();
    }

    // ---------------------------------------------------------
    // STATE MACHINE — YOUR EDIT AREA FOR INTAKE/OUTTAKE LOGIC
    // ---------------------------------------------------------
    public void update() {
        TelemetryPacket packet = new TelemetryPacket();

        switch (state) {

            case STEP_1:
                runIntakeLogic(1);
                runningAction = toP1.run(packet);

                if (!runningAction && timer.milliseconds() > 8000) {
                    state = AutoState.STEP_2;
                    timer.reset();
                    timer.startTime();
                    robot.outtake.setLinkage(0.92);
                    robot.intake.setPower(0);
                }
                break;

            case STEP_2:
                //runIntakeLogic(2);
                runningAction = toP2.run(packet);
                if (!runningAction) {
                    timer.reset();
                    state = AutoState.STEP_3;
                }
                break;

            case STEP_3:
                runIntakeLogic(3);
                runningAction = toP3.run(packet);
                if (!runningAction) {
                    timer.reset();
                    state = AutoState.STEP_4;
                }
                break;

            case STEP_4:
                runIntakeLogic(4);
                runningAction = toP4.run(packet);
                if (!runningAction) {
                    timer.reset();
                    state = AutoState.STEP_5;
                }
                break;

            case STEP_5:
                runIntakeLogic(5);
                runningAction = toP5.run(packet);
                if (!runningAction) {
                    timer.reset();
                    state = AutoState.STEP_6;
                }
                break;

            case STEP_6:
                runIntakeLogic(6);
                runningAction = toP6.run(packet);
                if (!runningAction) {
                    timer.reset();
                    state = AutoState.STEP_7;
                }
                break;

            case STEP_7:
                runIntakeLogic(7);
                runningAction = toP7.run(packet);
                if (!runningAction) {
                    timer.reset();
                    state = AutoState.STEP_8;
                }
                break;

            case STEP_8:
                runIntakeLogic(8);
                runningAction = toP8.run(packet);
                if (!runningAction) {
                    state = AutoState.END;
                }
                break;

            case END:
                runIntakeLogic(6767); // final stop
                break;
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    // ---------------------------------------------------------
    //  EASY TO EDIT INTAKE / OUTTAKE LOGIC
    // ---------------------------------------------------------
    public void runIntakeLogic(int step) {

        // -----------------------------
        // EXAMPLE — DELETE & CUSTOMIZE
        // -----------------------------
        switch (step) {
            case 1:
                if(timer.milliseconds() < 3000) {
                    robot.outtake.setVelocity(2500);
                }

                else if(timer.milliseconds() < 3500) {
                    robot.outtake.setLinkage(0.6);
                    robot.intake.setPower(0);
                }


                else if(timer.milliseconds() < 4500){
                    robot.outtake.setLinkage(0.6);
                    robot.intake.setPower(0.6);
                }

                else if(timer.milliseconds()<5500){
                    robot.intake.setPower(0);
                    robot.outtake.setVelocity(2500);
                }else{
                    robot.intake.setPower(0.8);
                }
                break;

            case 2:
                robot.intake.setPower(0);
                robot.outtake.spinToRpm(1500);
                break;

            case 3:
                robot.outtake.setLinkage(0.92);
                robot.intake.setPower(0.8);
                robot.outtake.setPower(-0.5);
                break;

            case 4:
                robot.outtake.setVelocity(2500);
                break;

            case 5:
                timer.reset();
                if(timer.milliseconds() < 100)
                    robot.outtake.setLinkage(0.6);
                else if(timer.milliseconds() < 1000)
                    robot.intake.setPower(1.0);
                else
                    robot.outtake.setLinkage(0.92);
                break;

            case 6:
                robot.intake.setPower(1);
                robot.outtake.setPower(-0.4);
                break;

            case 7:
                robot.outtake.setVelocity(2700);
                break;

            case 8:
                robot.intake.setPower(0);
                robot.outtake.setPower(0);
                break;

            default:
                robot.intake.setPower(0);
                robot.outtake.setPower(0);
                robot.outtake.spinToRpm(0);
        }
    }

    // ---------------------------------------------------------
    // OPMODE ENTRY POINT
    // ---------------------------------------------------------
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry, START);

        buildPaths();
        robot.init();

        waitForStart();
        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            robot.update(gamepad2);
            update();

            telemetry.addData("State", state);
            telemetry.update();
        }
    }
}
