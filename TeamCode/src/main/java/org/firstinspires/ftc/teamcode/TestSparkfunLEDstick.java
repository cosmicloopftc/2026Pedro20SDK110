package org.firstinspires.ftc.teamcode;
/*
        Copyright (c) 2021-24 Alan Smith

        All rights reserved.

        Redistribution and use in source and binary forms, with or without modification,
        are permitted (subject to the limitations in the disclaimer below) provided that
        the following conditions are met:

        Redistributions of source code must retain the above copyright notice, this list
        of conditions and the following disclaimer.

        Redistributions in binary form must reproduce the above copyright notice, this
        list of conditions and the following disclaimer in the documentation and/or
        other materials provided with the distribution.

        Neither the name of Alan Smith nor the names of its contributors may be used to
        endorse or promote products derived from this software without specific prior
        written permission.

        NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
        LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
        "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
        THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
        ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
        FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
        DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
        SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
        CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
        TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
        THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * This OpMode illustrates how to use the SparkFun QWIIC LED Strip
 *
 * This is a simple way to add a strip of 10 LEDs to your robot where you can set the color of each
 * LED or the whole strip.   This allows for driver feedback or even just fun ways to show your team
 * colors.
 *
 * Why?
 * Because more LEDs == more fun!!
 *
 * This OpMode assumes that the QWIIC LED Stick is attached to an I2C interface named "back_leds" in the robot configuration.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * You can buy this product here:  https://www.sparkfun.com/products/18354
 * Don't forget to also buy this to make it easy to connect to your Control or Expansion Hub:
 * https://www.sparkfun.com/products/25596
 */
@TeleOp(name = "Sparkfun LED Stick", group = "Test")
//@Disabled
public class TestSparkfunLEDstick extends OpMode {
    private boolean wasUp;
    private boolean wasDown;
    private int brightness = 5;  // this needs to be between 0 and 31
    private final static double END_GAME_TIME = 120 - 20;

    private SparkFunLEDStick ledStick;

    //*Setup IMU sensor object.
    public IMU imu;

    // Create Orientation variable
    Orientation myRobotOrientation;
    // Create angular velocity array variable
    AngularVelocity myRobotAngularVelocity;

    @Override
    public void init() {
        ledStick = hardwareMap.get(SparkFunLEDStick.class, "LEDstick");
        ledStick.setBrightness(brightness);
        ledStick.setColor(Color.GREEN);

        //map IMU
        //TODO: Define and initialize IMU sensor on new Control Hub--new orientation of the control hub
        imu = hardwareMap.get(IMU.class, "imu");
        //Define how the hub is mounted on robot to get correct Yaw, Pitch and Roll values.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        //Initialize IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        telemetry.addData("Status" , "Initialized");
        telemetry.update();

    }


    @Override
    public void start() {
        resetRuntime();
    }

    @Override
    public void loop() {
        telemetry.addLine("Hold the A button to turn blue");
        telemetry.addLine("Hold the B button to turn red");
        telemetry.addLine("Hold the left bumper to turn off");
        telemetry.addLine("Use DPAD Up/Down to change brightness");
        
        if (getRuntime() > END_GAME_TIME) {
            int[] ledColors = {Color.RED, Color.YELLOW, Color.RED, Color.YELLOW, Color.RED,
                    Color.YELLOW, Color.RED, Color.YELLOW, Color.RED, Color.YELLOW};
            ledStick.setColors(ledColors);
        } else if (gamepad1.a) {
            ledStick.setColor(Color.BLUE);
        } else if (gamepad1.b) {
            ledStick.setColor(Color.RED);
        } else if (gamepad1.left_bumper) {
            ledStick.turnAllOff();
        } else {
            ledStick.setColor(Color.GREEN);
        }

        /*
         * Use DPAD up and down to change brightness
         */
        int newBrightness = brightness;
        if (gamepad1.dpad_up && !wasUp) {
            newBrightness = brightness + 1;
        } else if (gamepad1.dpad_down && !wasDown) {
            newBrightness = brightness - 1;
        }
        if (newBrightness != brightness) {
            brightness = Range.clip(newBrightness, 0, 31);
            ledStick.setBrightness(brightness);
        }
        telemetry.addData("Brightness", brightness);


        // Read Angular Velocities
        myRobotAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        // Get Robot Orientation
        myRobotOrientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES
        );

// Then read or display the desired values (Java type float):
        float X_axis = myRobotOrientation.firstAngle;
        float Y_axis = myRobotOrientation.secondAngle;
        float Z_axis = myRobotOrientation.thirdAngle;

// Then read or display these values (Java type float)
// from the object you just created:
        float zRotationRate = myRobotAngularVelocity.zRotationRate;
        float xRotationRate = myRobotAngularVelocity.xRotationRate;
        float yRotationRate = myRobotAngularVelocity.yRotationRate;

        wasDown = gamepad1.dpad_down;
        wasUp = gamepad1.dpad_up;
    }
}
