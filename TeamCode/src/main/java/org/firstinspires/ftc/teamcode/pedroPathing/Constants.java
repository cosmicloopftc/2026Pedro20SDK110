package org.firstinspires.ftc.teamcode.pedroPathing;

/** 9/28/2025 convert old Pedropath LConstants and
 *      Fconstants from 2/1/2025 IntoTheDeep Robot to Constants here.

    1/18/2026
        robot frame length = 396 mm   (mid = 198 mm)
        robot frame width = 412 mm    (mid = 206 mm)
             forwardPodY(inch) = 3.15;  // [(206-126)=98/25.4 inch left of robot center] = inch from center to side of robot, positive is to toward left looking down on robot) =
            .strafePodX(inch) = 0;      // (198-199 mm)/25.4 = inch from center to bottom of robot, positive is to toward front looking down on robot) =
 */




import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.8)
            .forwardZeroPowerAcceleration(-37.97504)        //.forwardZeroPowerAcceleration(-31)
            .lateralZeroPowerAcceleration(-82.58920666)        //.lateralZeroPowerAcceleration(-69)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.1, 0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.06, 0, 0.0015, 0.6, 0)
            )
            .secondaryDrivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.008, 0, 0.0001, 0.6, 0)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)

//            .leftFrontMotorDirection(DcMotorEx.Direction.FORWARD)
//            .leftRearMotorDirection(DcMotorEx.Direction.FORWARD)
//            .rightFrontMotorDirection(DcMotorEx.Direction.REVERSE)
//            .rightRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .xVelocity(74.1846189333)    //.xVelocity(60)
            .yVelocity(55.5325466667);    //.yVelocity(41.5);

//    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .forwardPodY(5.25)
//            .strafePodX(-1.25)
//            .distanceUnit(DistanceUnit.INCH)
//            .hardwareMapName("pinpoint")
//            .encoderResolution(
//                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
//            )
//            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
//            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995, 100, 0.85, 0.8
    );

    //example constants on PedroPathing site 1/18/2026:  0.995,0.1,0.1,0.009,50,1.25,10,1
    //Dominic version: 0.995, 500, 1,1
    //new for PedroPath 2.0.2 to latest 2.0.5:  (0.99, 100, 1, 1);
    //brakingStrength default = 1;  lower allow for early slowdown/better accurate stop
    //      but slight more time for path.
    //      optimal tradeoff is 0.85; 0.8 is best for exact stop.
    //brakingStart default = 1;   I think it control stopping at end of entire path.
    //      higher means it does not stop at the end of path on time and overshoot
    //      1.0 = overshoot much at end of path
    //      0.9 =
    //      0.8 = seem okay
    //      0.6 = still overshoot
    //tValueConstraint = % accuracy of path--end point, so after overshooting, it jumps back to target point at the end.
    //      0.995 = better.


    /** OLD PedroPath:
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
    **/


//    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
//            .forwardTicksToInches(.001989436789)
//            .strafeTicksToInches(.001989436789)
//            .turnTicksToInches(.001989436789)
//            .leftPodY(1)
//            .rightPodY(-1)
//            .strafePodX(-2.5)
//            .leftEncoder_HardwareMapName("leftFront")
//            .rightEncoder_HardwareMapName("rightRear")
//            .strafeEncoder_HardwareMapName("rightFront")
//            .leftEncoderDirection(Encoder.FORWARD)
//            .rightEncoderDirection(Encoder.FORWARD)
//            .strafeEncoderDirection(Encoder.FORWARD)
//            .IMU_HardwareMapName("imu")
//            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.15)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            //.hardwareMapName("odom")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            //.forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)    //confirmed
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);    //confirmed






    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                //.threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
