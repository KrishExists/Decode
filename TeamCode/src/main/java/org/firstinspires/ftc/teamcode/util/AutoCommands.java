package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

public class AutoCommands {

    private static DcMotorEx transfer;

    private Robot robot;
    private Telemetry telem;
    private HardwareMap hw;

    public AutoCommands(DcMotorEx transfer, Robot robot1) {
        transfer = hw.get(DcMotorEx.class, "Transfer");
        robot = new Robot(hw, telem);
    }

    public static void prepareToShoot() {
        robot.intake.setPower(TeamConstants.INTAKE_FEED_POWER);
        transfer.setPower(TeamConstants.TRANSFER_REV);
        robot.outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
    }

    public static void spinUpIntake() {
        robot.outtake.spinToRpm(TeamConstants.outtake_Stop);
        robot.intake.setPower(TeamConstants.INTAKE_IN_POWER);
        transfer.setPower(TeamConstants.TRANSFER_CLOSED);
    }

    public static void spinUpShooter() {
        robot.outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
        robot.outtake.setLinkage(TeamConstants.LINKAGE_SHOOT);
    }
}
