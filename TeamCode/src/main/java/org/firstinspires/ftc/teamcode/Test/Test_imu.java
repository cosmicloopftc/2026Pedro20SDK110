package org.firstinspires.ftc.teamcode.Test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * This file demonstrates the impact of setting the IMU orientation correctly or incorrectly. This
 * code assumes there is an IMU configured with the name "imu".
 * <p>
 * Note: This OpMode is more of a tool than a code sample. The User Interface portion of this code
 *       goes beyond simply showing how to interface to the IMU.<br>
 *       For a minimal example of interfacing to an IMU, please see the SensorIMUOrthogonal or SensorIMUNonOrthogonal sample OpModes.
 * <p>
 * This sample enables you to re-specify the Hub Mounting orientation dynamically by using gamepad controls.
 * While doing so, the sample will display how Pitch, Roll and Yaw angles change as the hub is moved.
 * <p>
 * The gamepad controls let you change the two parameters that specify how the Control/Expansion Hub is mounted. <br>
 * The first parameter specifies which direction the printed logo on the Hub is pointing. <br>
 * The second parameter specifies which direction the USB connector on the Hub is pointing. <br>
 * All directions are relative to the robot, and left/right is as viewed from behind the robot.
 * <p>
 * How will you know if you have chosen the correct Orientation? With the correct orientation
 * parameters selected, pitch/roll/yaw should act as follows:
 * <p>
 *   Pitch value should INCREASE as the robot is tipped UP at the front. (Rotation about X) <br>
 *   Roll value should INCREASE as the robot is tipped UP at the left side. (Rotation about Y) <br>
 *   Yaw value should INCREASE as the robot is rotated Counter Clockwise. (Rotation about Z) <br>
 * <p>
 * The Yaw can be reset (to zero) by pressing the Y button on the gamepad (Triangle on a PS4 controller)
 * <p>
 * The rotational velocities should follow the change in corresponding axes.
 */

@Configurable
@TeleOp(name="Test_IMU Orientation v1.0", group="Test")

public class Test_imu extends LinearOpMode {

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;

    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
    static int LAST_DIRECTION = logoFacingDirections.length - 1;
    static float TRIGGER_THRESHOLD = 0.2f;

    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;

    @Override public void runOpMode() throws InterruptedException {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up      our robot: right
        usbFacingDirectionPosition = 2; // Forward  our robot: up

        updateOrientation();

        boolean justChangedLogoDirection = false;
        boolean justChangedUsbDirection = false;

        // Loop until stop requested
        while (!isStopRequested()) {

            // Check to see if Yaw reset is requested (Y button)
            if (gamepad1.y) {
                telemetryM.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetryM.addData("Yaw", "Press Y (triangle) on Gamepad to reset.\n");
            }

            // Check to see if new Logo Direction is requested
            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                if (!justChangedLogoDirection) {
                    justChangedLogoDirection = true;
                    if (gamepad1.left_bumper) {
                        logoFacingDirectionPosition--;
                        if (logoFacingDirectionPosition < 0) {
                            logoFacingDirectionPosition = LAST_DIRECTION;
                        }
                    } else {
                        logoFacingDirectionPosition++;
                        if (logoFacingDirectionPosition > LAST_DIRECTION) {
                            logoFacingDirectionPosition = 0;
                        }
                    }
                    updateOrientation();
                }
            } else {
                justChangedLogoDirection = false;
            }

            // Check to see if new USB Direction is requested
            if (gamepad1.left_trigger > TRIGGER_THRESHOLD || gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                if (!justChangedUsbDirection) {
                    justChangedUsbDirection = true;
                    if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
                        usbFacingDirectionPosition--;
                        if (usbFacingDirectionPosition < 0) {
                            usbFacingDirectionPosition = LAST_DIRECTION;
                        }
                    } else {
                        usbFacingDirectionPosition++;
                        if (usbFacingDirectionPosition > LAST_DIRECTION) {
                            usbFacingDirectionPosition = 0;
                        }
                    }
                    updateOrientation();
                }
            } else {
                justChangedUsbDirection = false;
            }

            // Display User instructions and IMU data
            telemetryM.addData("logo Direction (set with bumpers)", logoFacingDirections[logoFacingDirectionPosition]);
            telemetryM.addData("usb Direction (set with triggers)", usbFacingDirections[usbFacingDirectionPosition] + "\n");

            if (orientationIsValid) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

                double Yaw_Z = orientation.getYaw(AngleUnit.DEGREES);
                double Pitch_X = orientation.getPitch(AngleUnit.DEGREES);
                double Roll_Y = orientation.getRoll(AngleUnit.DEGREES);
                double Yaw_Z_velocity = angularVelocity.zRotationRate;
                double Pitch_X_veloxity = angularVelocity.xRotationRate;
                double Roll_Y_velocity = angularVelocity.yRotationRate;

                telemetryM.addData("Yaw (Z)", (double) Math.round(Yaw_Z) * 10 / 10);
                telemetryM.addData("Pitch (X)",(double) Math.round(Pitch_X) * 10 / 10);
                telemetryM.addData("Roll (Y)",(double) Math.round(Roll_Y) * 10 / 10);
                telemetryM.addData("Yaw (Z) velocity", (double) Math.round(Yaw_Z_velocity) * 10 / 10);
                telemetryM.addData("Pitch (X) velocity", (double) Math.round(Pitch_X_veloxity) * 10 / 10);
                telemetryM.addData("Roll (Y) velocity", (double) Math.round(Roll_Y_velocity) * 10 / 10);


//                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
//                telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
//                telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
//                telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
//                telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
//                telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            } else {
                telemetryM.addData("Error", "Selected orientation on robot is invalid");
            }

            //telemetry.update();
            telemetryM.update(telemetry);
        }
    }

    // apply any requested orientation changes.
    void updateOrientation() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }
    }
}