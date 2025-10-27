package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Static Servo Telemetry", group = "Telemetry")
public class clawTelemetry extends OpMode {

    // Declare servo objects
    //private ArmController armController;
    private Servo wristLeft;
    private Servo wristRight;

    private Servo linkage;
    private Servo pivotServo;
    private Servo clawServo;
    private Servo clawRotate;

    // Dashboard instance
    public static double wristpos = 0.0;
    public static double pivotServopos = 0.0;
    public static double clawServopos = 0.0;
    public static double clawRotatepos = 0.0;

    public static double linkagePos = 0.0;
    public static int armPosition = 0;

    @Override
    public void init() {
        // Initialize the hardware variables. The strings must match the names in the configuration

        // Initialize the dashboard
        // Initialize telemetry with both Driver Station and Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void loop() {
        linkage.setPosition(linkagePos);
        //armController.setArmPosition(armPosition);

        // Update the arm controller (PID and state machine updates)
        //armController.update();
        // Add servo positions to telemetry
        telemetry.addData("Linkage Position", linkage.getPosition());
        telemetry.update();

        // Send telemetry packet to the dashboard

    }
}
