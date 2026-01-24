package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;

import java.util.List;

//FTC Thunderbolts (S0acramento, CA) mentor program structure
//1:01:slider example adjusts the motor speed for the slide for it to reach a set Encoder value.
//OpMode structure: init(), init_loop(), start(), loop(), stop().  as of SDK 8.1, 1ms between call in loop()
//LinearOpMode structure: runOpMode(), waitForStart(), isStarted(), isStopRequested(), idle(), opModeIsActive(), opModeInInit()


@TeleOp (name= "MotorEncoder_Test", group = "Test")

public class MotorEncoderTest extends OpMode {
    //This method will be called once, when the INIT button is pressed.
    public static HardwareMain robot = new HardwareMain();

    ElapsedTime runtime = new ElapsedTime(); ;
    ElapsedTime lastButtonPushTimeA;
    ElapsedTime lastButtonPushTimeB;
    ElapsedTime timeLastPush = new ElapsedTime(); ;

    private boolean isPushed_dPad_up, isPushed_dPad_down;
    int  currentEncoder;


    public void init() {
        robot.init(hardwareMap);
        //Set up bulk data reading
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void init_loop () {

        telemetry.addData("Motor encoder value at start = ", robot.turretMotor.getCurrentPosition());
        telemetry.addLine("Press dPad_up to increase (Gamepad 1.");
        telemetry.addLine("Press dPad_down to increase.");
        telemetry.update();
        }

    public void start () {

            runtime.reset();
            timeLastPush =runtime;
        }


    public void loop () {
//        isPushed_dPad_up = gamepad1.dpadUpWasReleased();
//        isPushed_dPad_down = gamepad1.dpadDownWasPressed();
//
//
////        currentButtonStateB = gamepad1.dpad_down;
////
//        if (isPushed_dPad_up ) {
//            currentEncoder = currentEncoder + 10;
//            timeLastPush = runtime;
//            isPushed_dPad_up = false;
//            robot.turretMotor.setTargetPosition(currentEncoder);
//        }
//        if (isPushed_dPad_down) {
//            currentEncoder = currentEncoder - 10;
//            timeLastPush = runtime;
//            isPushed_dPad_down = false;
//            robot.turretMotor.setTargetPosition(currentEncoder);
//        }

        robot.turretMotor.setPower(gamepad1.left_stick_y);

        telemetry.addData("Motor encoder value at start = ", robot.turretMotor.getCurrentPosition());
        telemetry.update();


    }


    public void stop () {
    }


}
