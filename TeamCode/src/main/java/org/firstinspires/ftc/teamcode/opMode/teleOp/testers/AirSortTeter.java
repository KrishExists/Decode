

package org.firstinspires.ftc.teamcode.opMode.teleOp.testers;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
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


@TeleOp(name = "airtester", group = "Testers")
@Config
@Configurable
public class AirSortTeter extends LinearOpMode {
    private HardwareMap hw;
    private Outtake shooter;
    private DcMotorEx intake;

    private Robot robot;
    public static int current;
    public static int goal;
    public static boolean reset = true;
    private String[] cpattern;
    private String[] iea;
    int pos = 0;
    private boolean happend = false;
    private boolean secondcheck = false;

    private int count;
    private Servo blocker;
    private DcMotorEx transfer;
    public static double servo1;
    public static double servo2;

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
        goal = 0;
        current = 0;
        reset = true;
        shooter.linkage.setPosition(1);
        servo1 = 1;
        servo2 = 0.5;
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
//                blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
                shooter.setPower(0);
                intake.setPower(0);
                transfer.setPower(0);
            }else{
                shooter.spinToRpm(3500);
//                blocker.setPosition(TeamConstants.BLOCKER_OPEN);
                if(shooter.getRPM()>3400){
                    happend = true;
                    secondcheck = true;
                }
                if(happend){


                    //intake 1, transfer -1
                    intake.setPower(1);
                    transfer.setPower(1);
                    if(iea[count].equals(cpattern[pos])){
                        shooter.linkage.setPosition(servo2);
                        telemetry.addLine("0.55");

                        if(shooter.getRPM()<3250&&secondcheck){
                            count++;
                            pos++;
                            secondcheck = false;
                        }
                    }else{
                        shooter.linkage.setPosition(servo1);
                        telemetry.addLine("0.43");
                        if(shooter.getRPM()<3250){
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
            telemetry.addData("count",count);
            telemetry.addData("pos",pos);
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
