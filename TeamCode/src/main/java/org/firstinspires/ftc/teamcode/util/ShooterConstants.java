package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static double closeShootRPM = 1740; // 1800
    public static double farShootRPM = 2150; //2.16k
    public static double kf = 0.0, kp = 0.15, ki = 0, kd = 0.000001;
    public static double tuningTestingRPM = 0;
    public static int TICKS_PER_REV = 28;
    public static int MAX_RPM = 6000;

}
