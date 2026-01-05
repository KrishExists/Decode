
package org.firstinspires.ftc.teamcode.opMode.auto;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.AutoCommands;
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


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new RedAutoPaths(follower); // Build paths
        robot = new Robot(hardwareMap, telemetry);
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");


        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }



    public void spinUp(boolean withTransfer){
        AutoCommands.spinUpShooter();
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
            AutoCommands.prepareToShoot();
        }
        if(!follower.isBusy()) {
            if(robot.outtake.getRPM() > TeamConstants.Shooter_BottomThreshold|| happend) {
                happend = true;
                spinUp(true);
                if(timer.milliseconds() > 2000) {
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
            AutoCommands.prepareToShoot();
        }
        if(!follower.isBusy()) {

            if(robot.outtake.getRPM() > TeamConstants.Shooter_BottomThreshold||happend) {
                happend = true;
                spinUp(true);
                if(timer.milliseconds() > 2000) {
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
        AutoCommands.spinUpIntake();
        if(!follower.isBusy()) {
            follower.followPath(pathChain);
            pathState++;
            resetBooleans();
        }
    }
    public int autonomousPathUpdate() {
        switch (pathState){
            case 0:
                follower.followPath(paths.Path1);
                pathState = 1;
               spinUp(false);
               resetBooleans();
               happend = true;
            case 1:
                resetTime();
                shoot( paths.Path2);
            case 2:
                spinIntake(paths.Path3);
            case 3:
                spinIntake(paths.Path4);
            case 4:
                resetTime();
                shoot(paths.Path5);
            case 5:
                spinIntake(paths.Path6);
            case 6:
                spinIntake(paths.Path7);
            case 7:
                resetTime();
                shoot(paths.Path8);
            case 8:
                spinIntake(paths.Path9);
            case 9:
                spinIntake(paths.Path10);
            case 10:
                resetTime();
                shoot(paths.Path1,true);
        }
        // Event markers will automatically trigger at their positions
        // Make sure to register NamedCommands in your RobotContainer
        return pathState;
    }


}
    