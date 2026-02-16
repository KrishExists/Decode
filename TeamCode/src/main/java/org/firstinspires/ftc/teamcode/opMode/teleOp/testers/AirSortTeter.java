

package org.firstinspires.ftc.teamcode.opMode.teleOp.testers;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.TeamConstants;


@TeleOp(name = "airtester", group = "Main")
@Config
public class AirSortTeter extends LinearOpMode {
    private HardwareMap hw;
    private Outtake shooter;
    private DcMotorEx intake;

    private Robot robot;
    private static int current;
    private static int goal;
    private static boolean reset = true;
    private String[] cpattern;
    private String[] iea;
    int pos = 0;
    private boolean happend = false;
    private boolean secondcheck = false;

    private int count;
    private Servo blocker;
    private DcMotorEx transfer;

    @Override
    public void runOpMode() throws InterruptedException {


// ===== Initialize Hardware & Subsystems =====
        hw = hardwareMap;
        shooter = new Outtake(hw, telemetry);
        blocker = hardwareMap.get(Servo.class, "blocker");
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        telemetry.addLine("Initialized — Waiting for Start");
        telemetry.update();
        robot.init();
        waitForStart();
        if (isStopRequested()) return;


// ===== Main Loop =====
        while (opModeIsActive()) {
            if(reset){
                cpattern = getBallArray(current);
                iea = getBallArray(goal);
                happend = false;
                count = 0;
                pos = 0;
                secondcheck = false;
                blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
            }else{
                shooter.spinToRpm(2400);
                blocker.setPosition(TeamConstants.BLOCKER_OPEN);
                if(shooter.getRPM()>2300){
                    happend = true;
                    secondcheck = true;
                }
                if(happend){


                    //intake 1, transfer -1
                    intake.setPower(1);
                    transfer.setPower(-1);
                    if(iea[count].equals(cpattern[pos])){
                        shooter.linkage.setPosition(0.6);
                        if(shooter.getRPM()<2250&&secondcheck){
                            count++;
                            pos++;
                            secondcheck = false;
                        }
                    }else{
                        shooter.linkage.setPosition(0.47);
                        if(shooter.getRPM()<2250){
                            count++;
                        }
                    }

                }
            }
            if(count==3){

                count = 0;
                telemetry.addLine("count 3");
            }
            if(pos==3){
                pos = 0;
                telemetry.addLine("pos 3");
            }
            telemetry.addData("Shooter RPM", shooter.getRPM());
            telemetry.update();



        }
    }

    private String[] getBallArray(int goal) {
        String[] balls = {"P", "P", "P"}; // start with all purple
        if (goal >= 0 && goal < balls.length) {
            balls[goal] = "G"; // place green ball at goal position
        }
        return balls;
    }


}
