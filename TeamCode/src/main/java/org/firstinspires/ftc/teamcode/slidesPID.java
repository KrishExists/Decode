package org.firstinspires.ftc.teamcode;
//import android.health.connect.datatypes.units.Power;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class slidesPID extends OpMode{
    PIDController controller;
    ArmController arm;
    public static double kp = 0.008;
    public static double ki = 0;
    public static double kd = 0.00;
    public static double target = 0;
    public static double armPos = 0;
    private DcMotorEx slidesMotor;
    @Override
    public void init(){
        controller = new PIDController(kp,ki,kd,0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm = new ArmController(hardwareMap, false);
    }
    @Override
    public void loop(){
        int slidesPos = slidesMotor.getCurrentPosition();
        controller.setCoeffs(kp,ki,kd,0);
        double power = controller.calculate(target, slidesPos);
        slidesMotor.setPower(power);
        arm.setArmPosition(armPos);
        arm.update();
        telemetry.addData("Target: ", target);
        telemetry.addData("Pos", slidesPos);
        telemetry.addData("Power", slidesMotor.getPower());
    }
}
