package org.firstinspires.ftc.teamcode.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;


@TeleOp(name = "MainTeleDuplicate", group = "Main")
public class MainTeleDuplicate extends LinearOpMode {


    private Hardware hw;
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Vision vision;


    @Override
    public void runOpMode() throws InterruptedException {


// ===== Initialize Hardware & Subsystems =====
        hw = new Hardware(hardwareMap);
        shooter = new Shooter(hardwareMap);
        drive = new Drive(hw);
        intake = new Intake(hw, shooter);
        vision = new Vision(hw);


        telemetry.addLine("Initialized — Waiting for Start");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;


// ===== Main Loop =====
        while (opModeIsActive()) {


// Intake State Machine Update
            intake.update(gamepad2);


// Drive: Auto Align OR Manual
            if (gamepad2.left_bumper) {
                vision.autoAlignToTag(drive);
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