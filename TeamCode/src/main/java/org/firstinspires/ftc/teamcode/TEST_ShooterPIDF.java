package org.firstinspires.ftc.teamcode;

/** copy over from Marcus' branch 1/18/2026
 */


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

//FTC Thunderbolts (S0acramento, CA) mentor program structure
//1:01:slider example adjusts the motor speed for the slide for it to reach a set Encoder value.
//OpMode structure: init(), init_loop(), start(), loop(), stop().  as of SDK 8.1, 1ms between call in loop()
//LinearOpMode structure: runOpMode(), waitForStart(), isStarted(), isStopRequested(), idle(), opModeIsActive(), opModeInInit()


@TeleOp (name= "TEST_ShooterPIDF", group = "Test")

public class TEST_ShooterPIDF extends OpMode {
    //This method will be called once, when the INIT button is pressed.
    public DcMotorEx rightShooterMotor;
    public DcMotorEx leftShooterMotor;

    public double highVelocity = 1500;
    public double lowVelocity = 900;
    double curTargetVelocity = highVelocity;
    double F = 0;
    double P = 0;
    double [] stepSizes ={10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;


//    boolean lastButtonStateA = false;
//    boolean lastButtonStateB = false;
//
//    boolean changeVelocityIsActiveA = false;
//    boolean changeVelocityIsActiveB = false;
//
//    boolean currentButtonStateA = false;
//    boolean currentButtonStateB = false;



    public void init() {

        //Set up bulk data reading
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");

        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightShooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftShooterMotor.setDirection(DcMotorEx.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Init complete");

    }



    public void loop () {
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else { curTargetVelocity = highVelocity; }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        rightShooterMotor.setVelocity(curTargetVelocity);
        leftShooterMotor.setVelocity(curTargetVelocity);

        double curVelocityRightMotor = rightShooterMotor.getVelocity();
        double curVelocityLeftMotor = leftShooterMotor.getVelocity();
        double error = curTargetVelocity - curVelocityLeftMotor;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity Right Motor", "%.2f", curVelocityRightMotor);
        telemetry.addData("Current Velocity Left Motor", "%.2f", curVelocityLeftMotor);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("---------------------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

    }


//    public void stop () {
//        rightShooterMotor.setPower(0);
//        leftShooterMotor.setPower(0);
//    }


        //Algorithm to calculate speed? - this is temporary!!!!
//    private void shooterON(double RateDegPerSec){
//            //rightShooterMotor.setPower(-0.2);
//            //leftShooterMotor.setPower(0.2);    // 1/18/2026: Confirmed, motors spin in opposite to each others.
//            //TODO:  need to find optimal .setVelocity below
//            rightShooterMotor.setVelocity(RateDegPerSec, AngleUnit.DEGREES);       //value degrees per seconds
//            leftShooterMotor.setVelocity(-RateDegPerSec, AngleUnit.DEGREES);         //value degrees per seconds
//   //     leftShooterMotor.setVelocityPIDFCoefficients();
//        }

//   private void shooterOFF() {
//            rightShooterMotor.setPower(0);
//            leftShooterMotor.setPower(0);
//   }

//    public boolean isRisingEdgeA(boolean currentButtonStateA, boolean lastButtonStateB){
//        return currentButtonStateA && !lastButtonStateA;
//    }
//    public boolean isRisingEdgeB(boolean currentButtonStateB, boolean lastButtonStateB){
//        return currentButtonStateB && !lastButtonStateB;
//    }


}
