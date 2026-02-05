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


@TeleOp (name= "Shooter_Test", group = "Test")

public class ShooterTest extends OpMode {
    //Flywheel PF controller
    double P = 0;
    double I = 0;
    double D = 0;
    double F = 0;

    double highVelocity = 1600;
    double lowVelocity = 1280;
    double targetVelocity = highVelocity;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 0;

    //This method will be called once, when the INIT button is pressed.
    public DcMotorEx rightShooterMotor = null;
    public DcMotorEx leftShooterMotor = null;
    public Double AngularVel_DegPerSec;

//    boolean lastButtonStateA = false;
//    boolean lastButtonStateB = false;
//
//    boolean changeVelocityIsActiveA = false;
//    boolean changeVelocityIsActiveB = false;
//
//    boolean currentButtonStateA = false;
//    boolean currentButtonStateB = false;

    boolean isPushed_dPad_up = false;
    boolean isPushed_dPad_down = false;

    ElapsedTime runtime = new ElapsedTime(); ;
    ElapsedTime lastButtonPushTimeA;
    ElapsedTime lastButtonPushTimeB;
    ElapsedTime timeLastPush = new ElapsedTime(); ;

    public void init() {

        //Set up bulk data reading
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

            //map and setup mode of Shooter motor
            rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
            rightShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);        //use this for run at .setvelocity
            rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);               //use this for run at.setvelocity
            //rightShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);     //use this for run at .setpower
            rightShooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
            rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            rightShooterMotor.setPower(0);

            leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
            leftShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);     //use this for run at .setvelocity
            leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);            //use this for run at .setvelocity
            //leftShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);  //use this for run at .setpower
            leftShooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
            leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            leftShooterMotor.setPower(0);
        }





    }

    public void init_loop () {
        }

    public void start () {
            AngularVel_DegPerSec = -20.0;
            shooterON(AngularVel_DegPerSec);
            runtime.reset();
            timeLastPush =runtime;
        }


    public void loop () {
        isPushed_dPad_up = gamepad1.dpadUpWasReleased();
        isPushed_dPad_down = gamepad1.dpadDownWasPressed();


//        currentButtonStateB = gamepad1.dpad_down;
//
        if (isPushed_dPad_up ) {
            AngularVel_DegPerSec = AngularVel_DegPerSec - 5;
            timeLastPush = runtime;
            isPushed_dPad_up = false;
            shooterON(AngularVel_DegPerSec);
        }
        if (isPushed_dPad_down) {
            AngularVel_DegPerSec = AngularVel_DegPerSec + 5;
            timeLastPush = runtime;
            isPushed_dPad_down = false;
            shooterON(AngularVel_DegPerSec);
        }

        if (gamepad1.x) {
            shooterOFF();
        }

        if(gamepad2.aWasPressed()){
            if(targetVelocity == highVelocity){
                targetVelocity = lowVelocity;
            }else{
                targetVelocity = highVelocity;
            }
        }
        if(gamepad2.backWasPressed()){
            targetVelocity = 0;
        }
        if(gamepad2.yWasPressed()){
            stepIndex = (stepIndex +1) % stepSizes.length;
        }
        if(gamepad2.dpadUpWasPressed()){
            F += stepSizes[stepIndex];
        }
        if(gamepad2.dpadDownWasPressed()){
            F -= stepSizes[stepIndex];
        }
        if(gamepad2.dpadRightWasPressed()){
            P += stepSizes[stepIndex];
        }
        if(gamepad2.dpadLeftWasPressed()){
            P -= stepSizes[stepIndex];
        }

        if(gamepad2.bWasPressed()){
            I += stepSizes[stepIndex];
        }
        if(gamepad2.xWasPressed()){
            I -= stepSizes[stepIndex];
        }
        if(gamepad2.leftBumperWasPressed()){
            D += stepSizes[stepIndex];
        }
        if(gamepad2.rightBumperWasPressed()){
            D -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        rightShooterMotor.setVelocity(-targetVelocity);
        leftShooterMotor.setVelocity(targetVelocity);

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current velocity left", leftShooterMotor.getVelocity());
        telemetry.addData("Current velocity right", rightShooterMotor.getVelocity());
        telemetry.addData("Error left", targetVelocity - leftShooterMotor.getVelocity());
        telemetry.addData("Error right", targetVelocity + rightShooterMotor.getVelocity());
        telemetry.addLine("----------------------------------");
        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("F", F);
        telemetry.addData("Step size", stepSizes[stepIndex]);

//        telemetry.addData("rightShooter AngVel (deg/s?): ", rightShooterMotor.getVelocity(AngleUnit.DEGREES));
//        telemetry.addData("rightPower : ", rightShooterMotor.getPower());
//        telemetry.addLine(" ");
//        telemetry.addData("leftShooter  AngVel (deg/s?): ", leftShooterMotor.getVelocity(AngleUnit.DEGREES));
//        telemetry.addData("leftPower : ", leftShooterMotor.getPower());
//        telemetry.addLine(" ");
//        telemetry.addData("AngularVel_DegPerSec (deg/s?): ", AngularVel_DegPerSec);
    }


    public void stop () {
    }


        //Algorithm to calculate speed? - this is temporary!!!!
    private void shooterON(double RateDegPerSec){
            //rightShooterMotor.setPower(-0.2);
            //leftShooterMotor.setPower(0.2);    // 1/18/2026: Confirmed, motors spin in opposite to each others.
            //TODO:  need to find optimal .setVelocity below
            rightShooterMotor.setVelocity(RateDegPerSec, AngleUnit.DEGREES);       //value degrees per seconds
            leftShooterMotor.setVelocity(-RateDegPerSec, AngleUnit.DEGREES);         //value degrees per seconds
        }

   private void shooterOFF() {
            rightShooterMotor.setPower(0);
            leftShooterMotor.setPower(0);
   }

//    public boolean isRisingEdgeA(boolean currentButtonStateA, boolean lastButtonStateB){
//        return currentButtonStateA && !lastButtonStateA;
//    }
//    public boolean isRisingEdgeB(boolean currentButtonStateB, boolean lastButtonStateB){
//        return currentButtonStateB && !lastButtonStateB;
//    }


}
