

package org.firstinspires.ftc.teamcode.opMode.teleOp.testers;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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


@TeleOp(name = "LinDevloper", group = "Testers")
@Config
public class LinDevloper extends LinearOpMode {
    private HardwareMap hw;
    private Drivetrain drive;
    private Outtake shooter;
    private Intake intake;

    private Robot robot;
    public static double rpm = 0;
    public  DcMotorEx intakemotor;
    public  DcMotorEx transfermotor;
    private Servo linkage;
    private Servo blocker;
    public static double intakePower = 0;
    public static double transferPower  = 0;
    public static double linkagepos = 0;
    public static double blockerpos = 0;
    Servo turretServo;
    Servo turretServo2;


    @Override
    public void runOpMode() throws InterruptedException {


// ===== Initialize Hardware & Subsystems =====
        hw = hardwareMap;
        shooter = new Outtake(hw, telemetry);
        drive = new Drivetrain(hardwareMap, telemetry,true);
        telemetry.addLine("Initialized — Waiting for Start");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();
        rpm = 0;
        linkage = hardwareMap.get(Servo.class, "Linkage");
        blocker = hardwareMap.get(Servo.class, "blocker");
        transfermotor = hardwareMap.get(DcMotorEx.class, "Transfer");
        intakemotor = hardwareMap.get(DcMotorEx.class, "Intake");
        turretServo = hw.get(Servo.class, "TurretServo");
        turretServo2 = hw.get(Servo.class, "TurretServo2");
        turretServo.setPosition(0.5);
        turretServo2.setPosition(0.5);

        blockerpos = 0.58;
        blocker.setPosition(blockerpos);
        linkagepos = 0.47;
        linkage.setPosition(linkagepos);

        waitForStart();
        if (isStopRequested()) return;


// ===== Main Loop =====
        while (opModeIsActive()) {
            drive.update(gamepad1,gamepad2);
            double distance = drive.distanceToGoal();
            shooter.spinToRpm(rpm);
            blocker.setPosition(blockerpos);
            linkage.setPosition(linkagepos);
            intakemotor.setPower(intakePower);
            transfermotor.setPower(transferPower);
            telemetry.addData("Shooter RPM", shooter.getRPM());
            telemetry.addData("distance",distance);
            telemetry.addData("pose",drive.returnPose());

            telemetry.update();



        }
    }


}
