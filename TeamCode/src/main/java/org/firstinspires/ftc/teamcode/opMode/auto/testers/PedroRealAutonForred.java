
package org.firstinspires.ftc.teamcode.opMode.auto.testers;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.RedAutoPaths;
import org.firstinspires.ftc.teamcode.util.TeamConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroRealAutonForred extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    public RedAutoPaths paths; // Paths defined in the Paths class
    private Robot robot;
    private DcMotorEx transfer;
    private boolean ran = true;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean happend;
    private Pose startPose = new Pose(124,124,Math.toRadians(124));
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
public void BuildPaths(){
    Path1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(124.000, 124.000),

                            new Pose(96.000, 96.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))

            .build();

    Path2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(96.000, 96.000),
                            new Pose(96.000, 84.000),
                            new Pose(104.000, 84.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

            .build();

    Path3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(104.000, 84.000),

                            new Pose(124.000, 84.000)
                    )
            ).setTangentHeadingInterpolation()

            .build();

    Path4 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(124.000, 84.000),

                            new Pose(96.000, 96.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

            .build();

    Path5 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(96.000, 96.000),
                            new Pose(96.000, 60.000),
                            new Pose(105.000, 60.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

            .build();

    Path6 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(105.000, 60.000),

                            new Pose(124.000, 60.000)
                    )
            ).setTangentHeadingInterpolation()

            .build();

    Path7 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(124.000, 60.000),

                            new Pose(96.000, 96.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

            .build();

    Path8 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(96.000, 96.000),
                            new Pose(96.000, 36.000),
                            new Pose(105.000, 36.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

            .build();

    Path9 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(105.000, 36.000),

                            new Pose(124.000, 36.000)
                    )
            ).setTangentHeadingInterpolation()

            .build();

    Path10 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(124.000, 36.000),

                            new Pose(87.0000, 110.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

            .build();

}
    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Follower is busy",follower.isBusy());

        BuildPaths();
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
        pathState = 0;



    }

    @Override
    public void loop() {
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Follower is busy",follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    public  void prepareToShoot() {
        robot.intake.setPower(TeamConstants.INTAKE_FEED_POWER);
        transfer.setPower(TeamConstants.TRANSFER_REV);
        robot.outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
    }

    public  void spinUpIntake() {
        robot.outtake.spinToRpm(TeamConstants.outtake_Stop);
        robot.intake.setPower(TeamConstants.INTAKE_IN_POWER);
        transfer.setPower(TeamConstants.TRANSFER_CLOSED);
    }

    public  void spinUpShooter() {
        robot.outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
        robot.outtake.setLinkage(TeamConstants.LINKAGE_SHOOT);
    }

    public void spinUp(boolean withTransfer){
        spinUpShooter();
        if(withTransfer){
            transfer.setPower(TeamConstants.TRANSFER_REV);
            robot.intake.setPower(TeamConstants.INTAKE_IN_POWER);

        }

    }
    public void resetTime(){
        if(!follower.isBusy() && ran) {
            timer.reset();
            timer.startTime();
            ran =false;
        }
    }
    public void resetBooleans(){
        ran = true;
        happend = false;
    }
    public void shoot(PathChain nextPath,boolean skip){
        if(follower.isBusy()){
            prepareToShoot();
        }
        if(!follower.isBusy()) {
            if(robot.outtake.getRPM() > TeamConstants.Shooter_BottomThreshold|| happend) {
                happend = true;
                spinUp(true);
                if(timer.milliseconds() > 80000) {
                    if(skip){
                        pathState =67;
                        return;
                    }
                    follower.followPath(nextPath);
                    pathState++;
                    resetBooleans();
                }
            }else{
                spinUp(false);
            }

        }
    }
    public void shoot(PathChain nextPath){
        if(follower.isBusy()){
            prepareToShoot();
        }
        if(!follower.isBusy()) {

            if(robot.outtake.getRPM() > TeamConstants.Shooter_BottomThreshold||happend) {
                happend = true;
                spinUp(true);
                if(timer.milliseconds() > 80000) {
                    follower.followPath(nextPath);
                    pathState++;
                    resetBooleans();
                }
            }else{
                spinUp(false);
            }

        }
    }
    public void spinIntake(PathChain pathChain){
        spinUpIntake();
        if(!follower.isBusy()) {
            follower.followPath(pathChain);
            pathState++;
            resetBooleans();
        }
    }
    public int autonomousPathUpdate() {
        switch (pathState){
//            case 0:
//                follower.followPath(paths.Path1);
//                pathState = 1;
//               spinUp(false);
//               resetBooleans();
//               happend = true;
//            case 1:
//                resetTime();
//                shoot( paths.Path2);
//            case 2:
//                spinIntake(paths.Path3);
//            case 3:
//                spinIntake(paths.Path4);
//            case 4:
//                resetTime();
//                shoot(paths.Path5);
//            case 5:
//                spinIntake(paths.Path6);
//            case 6:
//                spinIntake(paths.Path7);
//            case 7:
//                resetTime();
//                shoot(paths.Path8);
//            case 8:
//                spinIntake(paths.Path9);
//            case 9:
//                spinIntake(paths.Path10);
//            case 10:
//                resetTime();
//                shoot(paths.Path1,true);
//        }
//        // Event markers will automatically trigger at their positions
//        // Make sure to register NamedCommands in your RobotContainer
//        return pathState;
            case 0:
                follower.followPath(Path1);
    }

return pathState;
}}
    