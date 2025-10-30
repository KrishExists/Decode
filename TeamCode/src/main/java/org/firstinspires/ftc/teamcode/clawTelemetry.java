package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Static Servo Telemetry", group = "Telemetry")
public class clawTelemetry extends OpMode {

    // Declare servo objects
    //private ArmController armController;


    private Servo linkage;
    private DcMotorEx shooter;
    private DcMotor intake;


    // Dashboard instance


    public static double linkagePos = 0.0;
    public static double shooterPos = 0.0;

    public static double intakePos = 0.0;

    @Override
    public void init() {
        // Initialize the hardware variables. The strings must match the names in the configuration
        linkage = hardwareMap.get(Servo.class, "Linkage");
        shooter = hardwareMap.get(DcMotorEx.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // Initialize the dashboard
        // Initialize telemetry with both Driver Station and Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void loop() {
        linkage.setPosition(linkagePos);
        shooter.setPower(shooterPos);
        intake.setPower(intakePos);
        //armController.setArmPosition(armPosition);

        // Update the arm controller (PID and state machine updates)
        //armController.update();
        // Add servo positions to telemetry
        telemetry.addData("Linkage Position", linkage.getPosition());
        telemetry.addData("Shooter Power", shooter.getVelocity()/28);
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.update();

        // Send telemetry packet to the dashboard

    }
}
