package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.control.DosaController;
import org.firstinspires.ftc.teamcode.control.DriveControl;
import org.firstinspires.ftc.teamcode.control.IntakeControl;
import org.firstinspires.ftc.teamcode.control.OuttakeControl;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Test", group = "")
public class OutakeTest extends LinearOpMode {

    Robot robot;
    DriveControl dc;
    IntakeControl ic;
    OuttakeControl oc;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private DosaController dosaController;
    private double kP = 0.03;  // Proportional constant for alignment
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public DcMotorEx outtake;
    public Servo linkage;

    public PIDController rpmPID;
    public double kp = 0.00625;
    public double ki = 0;
    public double kd = 0.001;

    public static double rpmThresh = 50;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        outtake.setDirection(DcMotorEx.Direction.REVERSE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        linkage = hardwareMap.get(Servo.class, "Linkage");

        rpmPID = new PIDController(kp, ki, kd);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("HI","shit");
            spinToRpm(5000);
            telemetry.addData("Velocity in ticks per second",getVelocity());
            telemetry.addData("Has gotten to good rpm",upToRpm(5000));
            telemetry.addData("Current rpm",currentRPM());
            telemetry.update();
        }

    }
    public void spinToRpm(double rpm) {
        rpmPID.setPID(kp, ki, kd);
        double output = rpmPID.calculate(currentRPM(), rpm);
        output = Range.clip(output, 0, 1);
        outtake.setPower(output);
    }

    public boolean upToRpm(double rpm) {
        double curr = currentRPM();
        return curr > rpm - rpmThresh && curr < rpm + rpmThresh;
    }

    public double currentRPM() {
        double conversion = 60 / (2 * Math.PI);
        return outtake.getVelocity(AngleUnit.RADIANS) * conversion;
    }

    // -- linkage --
    public void setLinkage(double pos) {
        linkage.setPosition(pos);
    }

    public double getVelocity() { return outtake.getVelocity();}

    public void setPower(double power){
        outtake.setPower(power);
    }
}
