package org.firstinspires.ftc.teamcode.pedroPathing;
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

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import android.graphics.Color;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;



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

@Configurable
@TeleOp(name = "TEST Sparkfun LED Stick v1.0", group = "Test")
//@Disabled

public class TestSparkfunLEDstick extends OpMode {
    private boolean wasUp;
    private boolean wasDown;
    private int brightness = 1;  // this needs to be between 0 and 31
    private final static double END_GAME_TIME = 30 - 20;
    private ElapsedTime endRuntime = new ElapsedTime();
    private ElapsedTime loopCycleTime = new ElapsedTime();

    private SparkFunLEDStick ledStick;
    int[] ledColors1 = {
            Color.RED, Color.YELLOW, Color.GREEN,
            Color.RED, Color.YELLOW, Color.GREEN,
            Color.RED, Color.YELLOW, Color.GREEN,
            Color.RED};
    int[] ledColors2 = {
            Color.YELLOW, Color.GREEN,Color.RED,
            Color.YELLOW, Color.GREEN, Color.RED,
            Color.YELLOW, Color.GREEN, Color.RED,
            Color.YELLOW};
    int[] ledColors3 = {
            Color.GREEN,Color.RED, Color.YELLOW,
            Color.GREEN,Color.RED, Color.YELLOW,
            Color.GREEN,Color.RED, Color.YELLOW,
            Color.GREEN};


    //*Setup IMU sensor object.
    public IMU imu;

    // Create Orientation variable
    Orientation myRobotOrientation;
    // Create angular velocity array variable
    AngularVelocity myRobotAngularVelocity;
    Acceleration myRobotAngularAccel;
    float xRotationRate, yRotationRate,zRotationRate;
    double xRotationRateBump, yRotationRateBump, zRotationRateBump =0.0;


    //----------Pedropath manual driving and incorporating PANEL-------------------------------
    public static Follower follower;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();


//    @Override
    public void onLog(List<String> lines) {}

    public static void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void draw() {
        Drawing.drawDebug(follower);
    }

    /** This creates a full stop of the robot by setting the drive motors to run at 0 power. */
    public static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
    }





    @Override
    public void init() {
        ledStick = hardwareMap.get(SparkFunLEDStick.class, "LEDstick");
        ledStick.turnAllOff();


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



        //----------Pedropath manual driving and incorporating PANEL-------------------------------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        poseHistory = follower.getPoseHistory();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    //----------Pedropath manual driving and incorporating PANEL-------------------------------
    /** This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry. */
    @Override
    public void init_loop() {
        ledStick.setBrightness(brightness);
        ledStick.setColor(Color.MAGENTA);

        telemetryM.addData("Status" , "Initialized");
        telemetryM.debug("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryM.addLine("");
        telemetryM.addLine("LED at init = MAGENTA");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();

    }

    @Override
    public void start() {
        resetRuntime();
        ledStick.setColor(Color.RED);

    //----------Pedropath manual driving and incorporating PANEL-------------------------------
        follower.startTeleopDrive();
        follower.update();
        

    }

    @Override
    public void loop() {
        loopCycleTime.reset();
        telemetryM.addData("RunTime: ", (double) Math.round(getRuntime()* 10) / 10);

        //----------Pedropath manual driving and incorporating PANEL-------------------------------
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
//        telemetryM.update(telemetry);
//        draw();


    //-----Sparkfun LED stick---------------------------------------------------------------------
        telemetryM.addLine("");
        telemetryM.addLine("Hold the A button to turn blue");
        telemetryM.addLine("Hold the B button to turn red");
        telemetryM.addLine("Hold the left bumper to turn off");
        telemetryM.addLine("Use DPAD Up/Down to change brightness");

        if (getRuntime() > END_GAME_TIME) {
            ledStick.setColors(ledColors1);
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

    //-----REV Control Hub imu info---------------------------------------------------------------------
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
        zRotationRate = myRobotAngularVelocity.zRotationRate;
        xRotationRate = myRobotAngularVelocity.xRotationRate;
        yRotationRate = myRobotAngularVelocity.yRotationRate;

        //set sensitivity to detect when robot is bumped
        if ((Math.abs(xRotationRate) > 300)
                || (Math.abs(yRotationRate) > 300)
                || (Math.abs(zRotationRate) > 300)) {
            xRotationRateBump = (double) Math.round(xRotationRate * 10) / 10;
            yRotationRateBump = (double) Math.round(yRotationRate * 10) / 10;
            zRotationRateBump = (double) Math.round(yRotationRate * 10) / 10;
        }

        wasDown = gamepad1.dpad_down;
        wasUp = gamepad1.dpad_up;

        telemetryM.addLine("");
        telemetryM.addData("Brightness",(double) Math.round(brightness * 10) / 10);
        telemetryM.addData("X_axis degree (pitch, front-to-back) ",(double) Math.round(X_axis * 10) / 10);
        telemetryM.addData("Y_axis degree (roll, side-to-side)",(double) Math.round(Y_axis * 10) / 10);
        telemetryM.addData("Z_axis degree (yaw, heading) ",(double) Math.round(Z_axis * 10) / 10);
        telemetryM.addLine("");
        telemetryM.addData("xRotationRate",(double) Math.round(xRotationRate * 10) / 10);
        telemetryM.addData("yRotationRate",(double) Math.round(yRotationRate * 10) / 10);
        telemetryM.addData("zRotationRate",(double) Math.round(zRotationRate * 10) / 10);
        telemetryM.addLine("");
        telemetryM.debug("BUMPED at RotationRate xyz: " + "  " +
                xRotationRateBump + " " + yRotationRateBump + " " + zRotationRateBump);


//        telemetryM.addData("Brightness", brightness);
//        telemetryM.addData("X_axis degree", X_axis);
//        telemetryM.addData("Y_axis degree", Y_axis);
//        telemetryM.addData("Z_axis degree", Z_axis);
//        telemetryM.addData("zRotationRate", zRotationRate);
//        telemetryM.addData("xRotationRate", xRotationRate);
//        telemetryM.addData("yRotationRate", yRotationRate);

        telemetryM.addLine("");
        telemetryM.debug("loopCycleTime (ms): " + (double) Math.round(loopCycleTime.milliseconds() * 100) / 100);
        telemetryM.update(telemetry);
        draw();
    }

    @Override
    public void stop() {
        ledStick.turnAllOff();
        ledStick.turnAllOff();
    }
}
