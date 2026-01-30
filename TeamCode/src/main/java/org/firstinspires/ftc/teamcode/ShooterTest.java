package org.firstinspires.ftc.teamcode;

/** copy over from Marcus' branch 1/18/2026
 */


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

//FTC Thunderbolts (S0acramento, CA) mentor program structure
//1:01:slider example adjusts the motor speed for the slide for it to reach a set Encoder value.
//OpMode structure: init(), init_loop(), start(), loop(), stop().  as of SDK 8.1, 1ms between call in loop()
//LinearOpMode structure: runOpMode(), waitForStart(), isStarted(), isStopRequested(), idle(), opModeIsActive(), opModeInInit()


@TeleOp (name= "TEST_Shooter", group = "Test")

public class ShooterTest extends OpMode {
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

    double timeToTarget1 = 0, timeToTarget2 = 0;
    boolean timeToTarget1Once = true, timeToTarget2Once = true;

    double maxVelocity = 0;

    public void init() {

        //Set up bulk data reading
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

            //map and setup mode of Shooter motor
            rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
            rightShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);        //use this for run at .setvelocity
            rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);               //use this for run at.setvelocity
            //rightShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);     //use this for run at .setpower
            rightShooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
            rightShooterMotor.setPower(0);

            leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
            leftShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);     //use this for run at .setvelocity
            leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);            //use this for run at .setvelocity
            //leftShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);  //use this for run at .setpower
            leftShooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
            leftShooterMotor.setPower(0);
        }





    }

    public void init_loop () {
        }

    public void start () {
            AngularVel_DegPerSec = -140.0;  // 140 for near, 190 for far
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

        shooterON(AngularVel_DegPerSec);

        if ((rightShooterMotor.getVelocity(AngleUnit.DEGREES) < -140) && (timeToTarget1Once)) {
            timeToTarget1 = runtime.milliseconds();
            timeToTarget1Once = false;
        }

        if ((rightShooterMotor.getVelocity(AngleUnit.DEGREES) > -145) && (!timeToTarget1Once) && (timeToTarget2Once)) {
            timeToTarget2 = runtime.milliseconds();
            timeToTarget2Once = false;
        }

        if ((rightShooterMotor.getVelocity(AngleUnit.DEGREES) < maxVelocity)) {
            maxVelocity = rightShooterMotor.getVelocity(AngleUnit.DEGREES);
        }

        telemetry.addData("rightShooter AngVel (deg/s?): ", rightShooterMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("rightPower : ", rightShooterMotor.getPower());
        telemetry.addLine(" ");
        telemetry.addData("leftShooter  AngVel (deg/s?): ", leftShooterMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("leftPower : ", leftShooterMotor.getPower());
        telemetry.addLine(" ");
        telemetry.addData("AngularVel_DegPerSec (deg/s?): ", AngularVel_DegPerSec);

        telemetry.addLine(" ");
        telemetry.addData("runTime (ms) = ", runtime.milliseconds());

        telemetry.addData("timeToTarget1 (ms) = ", timeToTarget1);
        telemetry.addData("timeToTarget2 (ms) = ", timeToTarget2);
        telemetry.addData("delta T2 - T1 (ms) = ", timeToTarget2 - timeToTarget1);
        telemetry.addData("maxVelocity (deg/s) = ", maxVelocity);

        telemetry.update();

    }


    public void stop () {
        rightShooterMotor.setPower(0);
        leftShooterMotor.setPower(0);
    }


        //Algorithm to calculate speed? - this is temporary!!!!
    private void shooterON(double RateDegPerSec){
            //rightShooterMotor.setPower(-0.2);
            //leftShooterMotor.setPower(0.2);    // 1/18/2026: Confirmed, motors spin in opposite to each others.
            //TODO:  need to find optimal .setVelocity below
            rightShooterMotor.setVelocity(RateDegPerSec, AngleUnit.DEGREES);       //value degrees per seconds
            leftShooterMotor.setVelocity(-RateDegPerSec, AngleUnit.DEGREES);         //value degrees per seconds
     //   leftShooterMotor.setVelocityPIDFCoefficients();
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
