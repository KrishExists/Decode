package org.firstinspires.ftc.teamcode.opMode.auto.testers;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.util.TeamConstants;




@Autonomous
@Config // Panels
public class NavaleOffseasonAutoBLUE extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private boolean happened = false;
    private boolean ran = false;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private DcMotorEx transfer;
    private ElapsedTime actionTimer;




    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144-124.006, 118.335, Math.toRadians(180-38)));

        paths = new Paths(follower);


        turret = new Turret(hardwareMap, telemetry, follower);
        //turret.auto(new Pose(144,144));


        outtake = new Outtake(hardwareMap, telemetry);
        outtake.linkage.setPosition(0.65);

        intake = new Intake(hardwareMap, telemetry, outtake, follower);
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");


        actionTimer = new ElapsedTime();


        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }
    @Override
    public void start(){
        setPathState(0);
        outtake.spinToRpm(1000);
    }


    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        //turret.auto(new Pose(144,144));


        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
    public void setPathState(int state) {
        pathState = state;
    }
    private void resetBooleans() {
        ran = false;
        happened = false;
    }


    public static class Paths {
        public PathChain scorePreload;
        public PathChain intakeSpike1;
        public PathChain scorePose1;
        public PathChain intakeSpike2;
        public PathChain scorePose2;
        public PathChain goGate;
        public PathChain scorePose3;
        public PathChain intakeSpike3;
        public PathChain scorePoseFINAL;


        public Paths(Follower follower) {
            scorePreload = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-124.006, 118.335),
                                    new Pose(144-97.30703331013615, 86.05199273888046)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-38), Math.toRadians(210))
                    .build();


            intakeSpike1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-97.30703331013615, 86.05199273888046),
                                    new Pose(144-125.3513335429328, 81.48395724129674)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();


            scorePose1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-125.3513335429328, 81.48395724129674),
                                    new Pose(144-91.520, 82.656)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(210))
                    .build();


            intakeSpike2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144-91.520, 82.656),
                                    new Pose(144-102.8554, 50.2826),
                                    new Pose(144-127.6, 59.9)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();


            scorePose2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-127.6, 59.9),
                                    new Pose(144-92.3, 83.3)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();


            goGate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144-92.3, 83.3),
                                    new Pose(144-116.13070438637367,56.4),
                                    new Pose(144-128.540, 58.7)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180-29))
                    .build();


            scorePose3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-128.540, 58.7),
                                    new Pose(144-87.82268554335522, 77.20421993622574)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-31), Math.toRadians(210))
                    .build();


            intakeSpike3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144-87.82268554335522, 77.20421993622574),
                                    new Pose(144-90.178, 29.710),
                                    new Pose(144-128.485, 34.802)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();


            scorePoseFINAL = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-128.485, 34.802),
                                    new Pose(144-91.806, 106)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }
    //    private void shoot(PathChain nextPath) {
//        shoot(nextPath, false);
//    }
//    //the different robot actions in each path lolz
//    private void spinUp(boolean withTransfer) {
//        if (withTransfer) {
//            transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
//            intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
//            telemetry.addLine("transfer at 1");
//
//        }else{
//            intake.setPower(0);
//            transfer.setPower(0);
//            telemetry.addLine("transfer at 0");
//        }
//    }
//
//    public void spinupIntake(){
//        intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
//        transfer.setPower(TeamConstants.TRANSFER_INTAKE_POWER);
//        outtake.setPower(0);
//    }
//    public void prepareToShoot(){
//        outtake.spinToRpm(3500);
//        intake.setPower(TeamConstants.INTAKE_STOP);
//        transfer.setPower(TeamConstants.TRANSFER_CLOSED);
//    }
//    private void spinIntake(PathChain nextPath) {
//        spinupIntake();
//        if (!ran) ran = follower.isBusy();
//        if (ran && !follower.isBusy()) {
//            follower.followPath(nextPath);
//            resetBooleans();
//            setPathState(pathState + 1);
//        }
//    }
//    private void shoot(PathChain nextPath, boolean skip) {
//        if (!happened) {
//            prepareToShoot();
//            actionTimer.reset();
//        }
//        turret.auto(new Pose(144,144));
//        if (!follower.isBusy()) {
//            if(!skip){
//            }
//            if ((outtake.atSpeed(3300,3600)||happened) ) {
//                happened = true;
//                telemetry.addData("RPM", outtake.getRPM());
//                spinUp(true);
//                if (actionTimer.milliseconds()>1200) {
//                    if (skip) {
//                        setPathState(99); //stops evt
//                        return;
//                    }
//                    follower.followPath(nextPath,true);
//                    setPathState(pathState + 1);
//                    resetBooleans();
//                }
//            } else {
//                telemetry.addLine("Outtake not above"); // Code never reaches here
//                spinUp(false);
//            }
//        }
//    }
    private void prepareToShoot() {
        intake.setPower(TeamConstants.INTAKE_STOP);
        outtake.spinToRpm(3400);
//        blocker.setPosition(TeamConstants.BLOCKER_OPEN);
        transfer.setPower(TeamConstants.TRANSFER_CLOSED);
    }

    private void spinUpIntake() {
        outtake.spinToRpm(TeamConstants.outtake_Stop);
        intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
        transfer.setPower(TeamConstants.TRANSFER_INTAKE_POWER);
    }

    private void spinUpShooter() {
        telemetry.addLine("Ready to shoot");
        outtake.spinToRpm(3400);
    }

    private void spinUp(boolean withTransfer) {
        spinUpShooter();
        if (withTransfer) {
            transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
            intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
            telemetry.addLine("transfer at 1");

        }else{
            intake.setPower(0);
            transfer.setPower(0);
            telemetry.addLine("transfer at 0");
        }
    }

    private void resetTimers() {
        if (!follower.isBusy() && ran) {
            actionTimer.reset();
            ran = false;
        }
    }

    private void shoot(PathChain nextPath, boolean skip) {
        if (follower.isBusy()) {
            prepareToShoot();
            actionTimer.reset();
        }
        turret.auto(new Pose(0,144));
        if (!follower.isBusy()) {
            if(!skip){
                //turret.auto(new Pose(144,144));
            }
            if ((outtake.atSpeed(3350,3450)||happened) ) {
                happened = true;
                spinUp(true);
                if (actionTimer.milliseconds()>1200) {
                    if (skip) {
                        pathState = 67;
                        return;
                    }
                    follower.followPath(nextPath,true);
                    pathState++;
                    resetBooleans();
                }
            } else {
                telemetry.addLine("Outtake not above"); // Code never reaches here
                spinUp(false);
            }
        }

    }

    private void shoot(PathChain nextPath) {
        shoot(nextPath, false);
    } // This is the shooting method, shoot then advance to the next path

    private void spinIntake(PathChain path,int y) {
        if(follower.isBusy()) {
            if (follower.getPose().getY() < y) {
                spinUpIntake();
            } else {
                outtake.spinToRpm(0);
                intake.setPower(0);
                transfer.setPower(0);
            }
        }
        if (follower.getCurrentTValue()>0.98) {
            follower.followPath(path,true);
            pathState++;
            resetBooleans();
        }
    }
    private void spinIntakeGate(PathChain path) {
        spinUpIntake();
        if (!follower.isBusy()&&actionTimer.milliseconds()>1500) {
            follower.followPath(path,true);
            pathState++;
            resetBooleans();
        }
    }


    public int autonomousPathUpdate() {
        switch (pathState) {


            case 0: // Begin driving to score preload
                follower.followPath(paths.scorePreload);
                setPathState(1);
                break;


            case 1: // Shoot preload, then follow intakeSpike1
                shoot(paths.intakeSpike1);
                break;


            case 2: // Driving to intakeSpike1 — spin up intake along the way
                spinIntake(paths.scorePose1, 250);
                break;


            case 3: // Shoot at scorePose1, then follow intakeSpike2
                shoot(paths.intakeSpike2);
                outtake.linkage.setPosition(1);
                break;


            case 4: // Driving to intakeSpike2 — spin up intake along the way
                spinIntake(paths.scorePose2, 250);
                break;


            case 5: // Shoot at scorePose2, then follow goGate
                shoot(paths.goGate);
                break;


            case 6: // Driving to gate — spin up intake along the way
                spinUpIntake();
                if (!ran) ran = follower.isBusy(); // wait until we confirm path is running
                if (ran && !follower.isBusy()) {
                    if(!happened){
                        happened = true;
                        actionTimer.reset(); //reseting the timer from the last shoot method
                    }
                    if (actionTimer.milliseconds() > 1500) {
                        follower.followPath(paths.scorePose3);
                        setPathState(7);
                    }
                }
                break;


            case 7: // Shoot at scorePose3, then follow intakeSpike3
                shoot(paths.intakeSpike3);
                break;


            case 8: // Driving to intakeSpike3 — spin up intake along the way
                spinIntake(paths.scorePoseFINAL, 42);
                break;


            case 9:
                // Final shot, no next path
                // turret.setPosition(0.2);
                outtake.linkage.setPosition(0.85);
                shoot(null, true);
                telemetry.addLine("Finished");
                break;
            default:
                outtake.setPower(0);
                intake.setPower(0);
                transfer.setPower(0);
                telemetry.addLine("Default");
                break;
        }
        return pathState;
    }
}


