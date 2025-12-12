package org.firstinspires.ftc.teamcode.opMode.teleOp;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Vision;


@TeleOp(name = "MainTeleDuplicate", group = "Main")
public class MainTeleDuplicate extends LinearOpMode {


    private Hardware hw;
    private Drivetrain drive;
    private Outtake shooter;
    private Intake intake;
    //private Vision vision;
    private boolean starts = true;


    @Override
    public void runOpMode() throws InterruptedException {


// ===== Initialize Hardware & Subsystems =====
        hw = new Hardware(hardwareMap);
        shooter = new Outtake(hardwareMap, telemetry);
        drive = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, shooter);
      //  vision = new Vision(hw);


        telemetry.addLine("Initialized — Waiting for Start");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        intake.timerStart();
        intake.timerReset();


// ===== Main Loop =====
        while (opModeIsActive()) {
//            if(starts){
//                intake.resetTime();
//                starts = false;
//            }//check if this is needed and works
// Intake State Machine Update
            intake.update(gamepad2);


// Drive: Auto Align OR Manual
            if (gamepad2.left_bumper) {
              //  vision.autoAlignToTag(drive);
            } else {
                drive.manualDrive(gamepad1);
            }

            if (gamepad1.a) {
                drive.alignToPose(new Pose2d(144, 144, Math.toRadians(90)));
            } else {
                drive.manualDrive(gamepad1);
            }



// Telemetry
            telemetry.addData("Intake State", intake.getState());
            telemetry.addData("Shooter RPM", shooter.atSpeed(0, 99999));
            telemetry.update();
        }
    }
}