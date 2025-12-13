package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Sensor Example")
public class ColorSensorTEst extends LinearOpMode {
    // Define the sensor
    RevColorSensorV3 colorSensor;

    @Override
    public void runOpMode() {
        // Initialize sensor using the name set in the configuration (e.g., "sensor_color")
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        // Optional: Gain helps the sensor see better in low light (default is 1)color
        colorSensor.setGain(2);

        waitForStart();

        while (opModeIsActive()) {
            // Get normalized colors (scaled 0.0 to 1.0)
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Read proximity (distance in CM)
            double distance = ((RevColorSensorV3) colorSensor).getDistance(DistanceUnit.CM);

            // Output data to the Driver Station
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);
            telemetry.addData("Distance (cm)", "%.3f", distance);
            telemetry.update();
        }
    }
}
