package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
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


    public Robot(HardwareMap h, Telemetry t, Drivetrain drive, Intake intake) {
        this.hardwareMap = h;
        this.telemetry = t;

        this.drive = drive;
        this.intake = intake;
        this.outtake = new Outtake(h,t);


        subsystems = List.of(drive, intake);
    }
    public Robot(HardwareMap h, Telemetry t) {
        this(h,t,new Drivetrain(h,t),new Intake(h,t, new Outtake(h,t)));
    }


    @Override
    public void init() {
        for(Subsystem s : subsystems){
            s.init();
        }
    }

    @Override
    public void update(Gamepad gamepad1,Gamepad gamepad2){
        for(Subsystem s : subsystems){
            s.update(gamepad1, gamepad2);
        }
    }
}
