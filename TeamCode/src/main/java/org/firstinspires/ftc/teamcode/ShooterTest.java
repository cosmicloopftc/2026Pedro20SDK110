package org.firstinspires.ftc.teamcode;

/** copy over from Marcus' branch 1/18/2026
 */


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

//FTC Thunderbolts (S0acramento, CA) mentor program structure
//1:01:slider example adjusts the motor speed for the slide for it to reach a set Encoder value.
//OpMode structure: init(), init_loop(), start(), loop(), stop().  as of SDK 8.1, 1ms between call in loop()
//LinearOpMode structure: runOpMode(), waitForStart(), isStarted(), isStopRequested(), idle(), opModeIsActive(), opModeInInit()


@TeleOp (name= "Shooter_Test")

public class ShooterTest extends OpMode {
    //This method will be called once, when the INIT button is pressed.
    public DcMotorEx rightShooterMotor = null;
    public DcMotorEx leftShooterMotor = null;
    public Double AngularVel_DegPerSec;
    boolean aPressedLastCycle = false;
    boolean bPressedLastCycle = false;

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
            AngularVel_DegPerSec = 5.0;
            shooterON(AngularVel_DegPerSec);
        }


    public void loop () {
        boolean aPressednow = gamepad1.dpad_up;
        boolean bPressednow = gamepad1.dpad_down;

        if (aPressednow && !aPressedLastCycle) {
            AngularVel_DegPerSec = AngularVel_DegPerSec - 5;

        }
        if (bPressednow && !bPressedLastCycle) {
            AngularVel_DegPerSec = AngularVel_DegPerSec + 5;

        }
        if (gamepad1.x) {
            shooterOFF();
        }
        shooterON(AngularVel_DegPerSec);

        telemetry.addData("rightShooter AngVel (deg/s?): ", rightShooterMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("leftShooter  AngVel (deg/s?): ", leftShooterMotor.getVelocity(AngleUnit.DEGREES));
    }


    public void stop () {
    }


        //Algorithm to calculate speed? - this is temporary!!!!
    public void shooterON(double RateDegPerSec){
            //rightShooterMotor.setPower(-0.2);
            //leftShooterMotor.setPower(0.2);    // 1/18/2026: Confirmed, motors spin in opposite to each others.
            //TODO:  need to find optimal .setVelocity below
            rightShooterMotor.setVelocity(-RateDegPerSec, AngleUnit.DEGREES);       //value degrees per seconds
            leftShooterMotor.setVelocity(RateDegPerSec, AngleUnit.DEGREES);         //value degrees per seconds
        }

   public void shooterOFF() {
            rightShooterMotor.setPower(0);
            leftShooterMotor.setPower(0);
   }


}
