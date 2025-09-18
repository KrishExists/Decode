package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;

@Config
@TeleOp(name = "Static Servo Telemetry", group = "Telemetry")
public class clawTelemetry extends OpMode {

    // Declare servo objects
    private ArmController armController;
    private Servo wristLeft;
    private Servo wristRight;
    private Servo pivotServo;
    private Servo clawServo;
    private Servo clawRotate;

    // Dashboard instance
    public static double wristpos = 0.0;
    public static double pivotServopos = 0.0;
    public static double clawServopos = 0.0;
    public static double clawRotatepos = 0.0;
    public static int armPosition = 0;

    @Override
    public void init() {
        // Initialize the hardware variables. The strings must match the names in the configuration
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        wristRight = hardwareMap.get(Servo.class, "wristRight");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        wristLeft.setDirection(Servo.Direction.REVERSE);
        // Initialize the dashboard
        armController = new ArmController(hardwareMap, true);
        // Initialize telemetry with both Driver Station and Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        armController.setArmPosition(armPosition);
    }


    @Override
    public void loop() {
        wristLeft.setPosition(wristpos);
        //wristRight.setPosition(wristpos);
        pivotServo.setPosition(pivotServopos);
        clawServo.setPosition(clawServopos);
        clawRotate.setPosition(clawRotatepos);
        armController.setArmPosition(armPosition);

        // Update the arm controller (PID and state machine updates)
        armController.update();
        // Add servo positions to telemetry
        telemetry.addData("wristLeft Position", wristLeft.getPosition());
        telemetry.addData("wristRight Position", wristRight.getPosition());
        telemetry.addData("pivotServo Position", pivotServo.getPosition());
        telemetry.addData("clawServo Position", clawServo.getPosition());
        telemetry.addData("clawRotate pos", clawRotate.getPosition());
        telemetry.addData("Arm Position", armController.getArmPosition());
        telemetry.update();

        // Send telemetry packet to the dashboard

    }
}
