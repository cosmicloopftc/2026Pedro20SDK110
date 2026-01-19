package org.firstinspires.ftc.teamcode.Hardware;

/** copy over from Marcus' branch 1/18/2026
 */


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure
//TODO: program the Servo position


public class HardwareIntake {

    public DcMotorEx intakeMotor = null;

    public CRServo intakeLeftTransfer = null;
    public CRServo intakeRightTransfer = null;

    //Servo Test
//    public Servo Servo_Test = null;


    /*Constructor*/
    public HardwareIntake() {
    }

    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        //Save reference to Hardware map

        //map and setup mode of Intake Slide Motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //intake slide motor behaviors
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setPower(0);

        //map and setup mode of Intake Servos
        intakeLeftTransfer = hardwareMap.get(CRServo.class, "intakeLeftTransfer");
        intakeRightTransfer = hardwareMap.get(CRServo.class, "intakeRightTransfer");

        //TEST SERVO
//        Servo_Test = hardwareMap.get(Servo.class, "Servo_Test");

    }


    public void start(){

    }


    public void stop () {

    }

    //TODO: Method for entire intake process
    //public method (function) for intaking in sample
    public void intakeIN() {
        intakeMotor.setPower(0.6);
    }


    //public method (function) for spitting out sample
    public void intakeOUT() {
        intakeMotor.setPower(-0.6);
    }

    //public method (function) for stopping the intake
    public void intakeSTOP() {
        intakeMotor.setPower(0);
    }
}