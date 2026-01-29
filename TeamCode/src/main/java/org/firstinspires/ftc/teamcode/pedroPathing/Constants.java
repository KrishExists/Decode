//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.control.FilteredPIDFCoefficients;
//import com.pedropathing.control.PIDFCoefficients;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.follower.FollowerConstants;
//import com.pedropathing.ftc.FollowerBuilder;
//import com.pedropathing.ftc.drivetrains.MecanumConstants;
//import com.pedropathing.ftc.localization.Encoder;
//import com.pedropathing.ftc.localization.constants.PinpointConstants;
//import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//@Configurable
//public class Constants {
//    public static FollowerConstants followerConstants = new FollowerConstants()
//            .mass(10.88);
////            .forwardZeroPowerAcceleration(-44.1422210)
////            .lateralZeroPowerAcceleration(-70.660132);
//    public static MecanumConstants driveConstants = new MecanumConstants()
//            .maxPower(1)
//            .rightFrontMotorName("frontRightMotor")
//            .rightRearMotorName("backRightMotor")
//            .leftRearMotorName("backLeftMotor")
//            .leftFrontMotorName("frontLeftMotor")
//
//            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
//
////            .xVelocity(56.41013077)
////            .yVelocity(39.5215982);
//
//    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .forwardPodY(5.6663)
//            .strafePodX(-2.3)//2
//            .distanceUnit(DistanceUnit.INCH)
//            .hardwareMapName("pinIMU")
//            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
//            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
//    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
//
//    public static Follower createFollower(HardwareMap hardwareMap) {
//        return new FollowerBuilder(followerConstants, hardwareMap)
//                .pathConstraints(pathConstraints)
//                .pinpointLocalizer(localizerConstants)
//                .mecanumDrivetrain(driveConstants)
//                .build();
//    }
//}

package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.88)
            .forwardZeroPowerAcceleration(-51.79219486028157)
            .lateralZeroPowerAcceleration(-66.1634807374069)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15,0,0.015,0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(1.25, 0, 0.1, 0.01));


    // heading is p at 0.5 and f 0.01

    /// secpnd d0.2 f 0.01  i 0 p 3

        public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                .xVelocity(61.116120586245074)
                .yVelocity(43.232616514671506);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.6663)
            .strafePodX(-2)//2
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinIMU")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}