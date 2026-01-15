

package org.firstinspires.ftc.teamcode.opMode.teleOp.testers;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Configurable
@Config
@TeleOp(name = "ester", group = "Main")
public class ester extends LinearOpMode {
    private HardwareMap hw;
    private Drivetrain drive;
    private Outtake shooter;
    private Intake intake;

    private Robot robot;
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {


// ===== Initialize Hardware & Subsystems =====
        hw = hardwareMap;
        shooter = new Outtake(hw, telemetry);
        drive = new Drivetrain(hardwareMap, telemetry,true);
        intake = new Intake(hw, telemetry, shooter);
        robot = new Robot(hw,telemetry,drive,intake);
           colorSensor = hardwareMap.get(ColorSensor.class, "colors");
        telemetry.addLine("Initialized — Waiting for Start");
        telemetry.update();
        robot.init();
        waitForStart();
        if (isStopRequested()) return;


// ===== Main Loop =====
        while (opModeIsActive()) {
            telemetry.update();

            telemetry.addData("Color Sensor Red: ", colorSensor.red());
            telemetry.addData("Color Sensor Blue: ", colorSensor.blue());
            telemetry.addData("Color Sensor Green: ", colorSensor.green());


        }
    }


}
