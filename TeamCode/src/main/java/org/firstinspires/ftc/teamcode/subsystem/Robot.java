package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Robot implements Subsystem { //The Rhetorical Situation

    public List<Subsystem> subsystems;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Drivetrain drive;
    public Intake intake;
    public Outtake outtake;

    public Robot(HardwareMap h, Telemetry t, Pose2d pose) {
        this.hardwareMap = h;
        this.telemetry = t;

        drive = new Drivetrain(h, t, pose);
        intake = new Intake(h, t);
        outtake = new Outtake(h, t);

        subsystems = List.of(drive, intake, outtake);
    }

    public Robot(HardwareMap h, Telemetry t) {
        this(h, t, new Pose2d(0, 0, 0));
    }

    @Override
    public void init() {
        for(Subsystem s : subsystems){
            s.init();
        }
    }

    @Override
    public void update() {
        for(Subsystem s : subsystems){
            s.update();
        }
    }
}
