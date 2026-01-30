package org.firstinspires.ftc.teamcode;

/** copy over from Marcus' branch 1/18/2026
 */


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;

//FTC Thunderbolts (S0acramento, CA) mentor program structure
//1:01:slider example adjusts the motor speed for the slide for it to reach a set Encoder value.
//OpMode structure: init(), init_loop(), start(), loop(), stop().  as of SDK 8.1, 1ms between call in loop()
//LinearOpMode structure: runOpMode(), waitForStart(), isStarted(), isStopRequested(), idle(), opModeIsActive(), opModeInInit()


@TeleOp (name= "Intake_Test", group = "Test")

public class IntakeTest extends OpMode {
    //This method will be called once, when the INIT button is pressed.

    public DcMotor shooterLeft = null;
    public DcMotor shooterRight = null;
    public static HardwareMain robot = new HardwareMain();

    public void init() {
        robot.init(hardwareMap);
    }

    public void init_loop() {
    }

    public void start(){

    }



    public void loop(){
        if (gamepad1.dpad_up){
            robot.transferUP();
            telemetry.addData("Transfer servo position", robot.transferServo.getPosition());
        }
        else if (gamepad1.dpad_down){
            robot.transferDOWN();
            telemetry.addData("Transfer servo position", robot.transferServo.getPosition());
        }
        else if (gamepad1.dpad_left || gamepad1.dpad_right){
            robot.shooterON();
        }else if(gamepad1.x){
            robot.spindexerPosition1();
        }else if(gamepad1.y){
            robot.spindexerPosition2();
        }else if(gamepad1.b){
            robot.spindexerPosition3();
        }else if(gamepad2.dpadLeftWasReleased()){
            robot.hoodServo.setPosition(robot.hoodServo.getPosition() + 0.01);
        }else if(gamepad2.dpadRightWasReleased()){
            robot.hoodServo.setPosition(robot.hoodServo.getPosition() - 0.01);
        }
        telemetry.addData("Hood servo position", robot.hoodServo.getPosition());
    }


    public void stop () {

    }

}