package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//FTC Thunderbolts (S0acramento, CA) mentor program structure
//1:01:slider example adjusts the motor speed for the slide for it to reach a set Encoder value.
//OpMode structure: init(), init_loop(), start(), loop(), stop().  as of SDK 8.1, 1ms between call in loop()
//LinearOpMode structure: runOpMode(), waitForStart(), isStarted(), isStopRequested(), idle(), opModeIsActive(), opModeInInit()


@TeleOp (name= "Intake_Test")

public class IntakeTest extends OpMode {
    //This method will be called once, when the INIT button is pressed.

    public DcMotor intakeMotor = null;

    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init_loop() {
    }

    public void start(){

    }



    public void loop(){
        if (gamepad1.dpad_up){
            intakeMotor.setPower(0.5);
        }
        else if (gamepad1.dpad_down){
            intakeMotor.setPower(-0.5);
        }
        else if (gamepad1.dpad_left || gamepad1.dpad_right){
            intakeMotor.setPower(0);
        }
    }


    public void stop () {

    }

}