package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

public class AutoCommands {

    private DcMotorEx transfer;

    private Robot robot;
    private Telemetry telem;
    private HardwareMap hw;

    public AutoCommands(DcMotorEx transfer, Robot robot1) {
        this.transfer = transfer;
        robot = robot1;
    }


}
