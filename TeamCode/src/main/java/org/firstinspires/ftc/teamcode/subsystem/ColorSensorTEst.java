package org.firstinspires.ftc.teamcode.subsystem;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Config
@TeleOp(name = "ColorSensorTester", group = "Main")
public class ColorSensorTest extends OpMode {

    TestBenchColor bench = new TestBenchColor();


    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        bench.getDetectedColor(telemetry);
        telemetry.update();
    }
}
