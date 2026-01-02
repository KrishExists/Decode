package org.firstinspires.ftc.teamcode.roadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadRunner.messages.TwoDeadWheelInputsMessage;

import java.util.Objects;

@Config
public final class CombinedLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = -3172.570740366367; // y position of the parallel encoder (in tick units)
        public double perpXTicks = 1155.8848344742353; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    // two wheel
    public final Encoder par, perp;
    public final IMU imu;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private final double inPerTick;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;
    private Pose2d pose;

    // pinpoint
    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public CombinedLocalizer(HardwareMap hardwareMap, IMU imu, double inPerTick, Pose2d initialPose) {
        // two wheel
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "backLeftMotor")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontLeftMotor")));

        par.setDirection(DcMotorSimple.Direction.REVERSE);
        perp.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: reverse encoder directions if needed
        //   par.setDirection(DcMotorSimple.Direction.REVERSE);

        this.imu = imu;

        this.inPerTick = inPerTick;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);

        pose = initialPose;

        // pinpoint
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinIMU");

        double mmPerTick = inPerTick * 25.4;
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);

        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();

        // Get the heading from Pinpoint only
        Rotation2d heading = new Rotation2d(0, 0);
        double rawHeadingVel = 0;
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            heading = Rotation2d.exp(driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            rawHeadingVel = driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        }

        // Correct wraparound for angular velocity
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        // Read encoders
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        // Encoder deltas (XY only, no heading correction)
        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading); // heading delta from Pinpoint only

        // Convert encoder deltas to inches
        double dx = parPosDelta * inPerTick;
        double dy = perpPosDelta * inPerTick;

        // Build twist: XY from dead wheels, heading from Pinpoint
        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[]{dx, parPosVel.velocity * inPerTick}),
                        new DualNum<>(new double[]{dy, perpPosVel.velocity * inPerTick})
                ),
                new DualNum<>(new double[]{headingDelta, headingVel})
        );

        // Update last readings
        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        // Apply pose update
        pose = pose.plus(twist.value());
        return twist.velocity().value();
    }

}
