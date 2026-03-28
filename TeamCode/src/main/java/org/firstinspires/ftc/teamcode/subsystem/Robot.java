//package org.firstinspires.ftc.teamcode.subsystem;
//
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import java.util.List;
//
//public class Robot implements Subsystem { //The Rhetorical Situation
//
//    public List<Subsystem> subsystems;
//
//    HardwareMap hardwareMap;
//    Telemetry telemetry;
//
//    public Drivetrain drive;
//    public Intake intake;
//    public Outtake outtake;
//
//
////    public Robot(HardwareMap h, Telemetry t, Drivetrain drive, Intake intake) {
////        this.hardwareMap = h;
////        this.telemetry = t;
////
////        this.drive = drive;
////        this.intake = intake;
////        this.outtake = new Outtake(h,t);
////
////
////        subsystems = List.of(drive, intake);
////    }
//
////    public Robot(HardwareMap h, Telemetry t) {
////        this(h,t,new Drivetrain(h,t),new Intake(h,t, new Outtake(h,t)));
////    }
//
//    public Robot(HardwareMap h, Telemetry t, Drivetrain drive, Intake intake, Turret turret) {
//        this.hardwareMap = h;
//        this.telemetry = t;
//
//        this.drive = drive;
//        this.intake = intake;
//
//        subsystems = List.of(drive, intake,turret);
//    }
//
//
//    public void init() {
//        intake.init();//inits evt
//    }
//
//    @Override
//    public void update(Gamepad gamepad1,Gamepad gamepad2){
//        for(Subsystem s : subsystems){
//            s.update(gamepad1, gamepad2);
//        }
//    }
//}
package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.StateMachine.Buttons;
import org.firstinspires.ftc.teamcode.StateMachine.StateMachine;

import java.util.List;

public class Robot implements Subsystem { //The Rhetorical Situation

    public List<Subsystem> subsystems;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Drivetrain drive;
    public Intake intake;
    public Outtake outtake;
    private StateMachine stateMachine;
    private Buttons gp1;
    private  Buttons gp2;


//    public Robot(HardwareMap h, Telemetry t, Drivetrain drive, Intake intake) {
//        this.hardwareMap = h;
//        this.telemetry = t;
//
//        this.drive = drive;
//        this.intake = intake;
//        this.outtake = new Outtake(h,t);
//
//
//        subsystems = List.of(drive, intake);
//    }

//    public Robot(HardwareMap h, Telemetry t) {
//        this(h,t,new Drivetrain(h,t),new Intake(h,t, new Outtake(h,t)));
//    }
    public Robot(HardwareMap h, Telemetry t, Drivetrain drive, Intake intake, Turret turret, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = h;
        this.telemetry = t;

        this.drive = drive;
        this.intake = intake;
        this.outtake = this.intake.shooter;

        subsystems = List.of(drive, intake,turret);
        gp1 = new Buttons(gamepad1);
        gp2 = new Buttons(gamepad2);
        stateMachine = new StateMachine(gp1,gp2);

        stateMachine.addState("Rest", "Main", s -> s
                .continuous()
                .onStart(this::stop)
                .nextOn(gp2.rt::isDown, "Intake")
                .nextOn(gp2.lt::isDown, "Outtake")
        )

        .addState("Intake", "Main", s -> s
                .continuous()
                .onStart(this::intake)
                .nextOn( gp2.rt::isUp, "Rest")
        )

        .addState("Outtake", "Main", s -> s
                .continuous()
                .onStart(() -> outtake.spinToRpm(2450))
                .onUpdate(this::shoot)
                .onStop(this::stop)
                .nextOn(gp2.lt::isUp, "Rest")
        )

        .addState("TurretMan", "Turret", s -> s
                .continuous()
                .onStart(turret::manual)
                .nextOn(gp2.lb::pressed, "TurretAuto")
        )

        .addState("TurretAuto", "Turret", s -> s
                .continuous()
                .onStart(() -> turret.auto(new Pose(144, 144)))
                .nextOn(gp2.lb::pressed, "TurretMan")
        );

        stateMachine.setInitialState("Main",   "Rest");
        stateMachine.setInitialState("Turret", "TurretMan");
        stateMachine.init();


    }
    public void intake(){
        intake.setPower(1);
        intake.transferpower(-0.5);
        outtake.setPower(0);
    }

    public void stop(){
        intake.setPower(0);
        intake.transferpower(0);
        outtake.setPower(0);
    }


    public void init() {
        intake.init();//inits evt
    }
    public void shoot(){
        if (outtake.atSpeed(2400, 2500)) {
            intake.setPower(1);
            intake.transferpower(1);
        } else {
            intake.setPower(0);
            intake.transferpower(0);
        }
    }
    @Override
    public void update(Gamepad gamepad1,Gamepad gamepad2){

        stateMachine.updateAll();
    }
}
