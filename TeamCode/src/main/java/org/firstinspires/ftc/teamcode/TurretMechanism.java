package org.firstinspires.ftc.teamcode;

/** copy over from Marcus' branch 1/18/2026
 */


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

//FTC Thunderbolts (S0acramento, CA) mentor program structure
//1:01:slider example adjusts the motor speed for the slide for it to reach a set Encoder value.
//OpMode structure: init(), init_loop(), start(), loop(), stop().  as of SDK 8.1, 1ms between call in loop()
//LinearOpMode structure: runOpMode(), waitForStart(), isStarted(), isStopRequested(), idle(), opModeIsActive(), opModeInInit()



public class TurretMechanism {

    private DcMotorEx turretMotor;

    private double kP = 0.0001;
    private double kD = 0.0000;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.2;
    private final double MAX_POWER = 0.6;
    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void setkP(double newKP) {
        kP = newKP;
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double newKD) {
        kD = newKD;
    }

    public double getkD() {
        return kD;
    }

    public void resetTimer(){
        timer.reset();
    }

    public void update(LLResult curID) {
        double deltaTime = timer.seconds();
        timer.reset();

        if (curID == null) {
            turretMotor.setPower(0);
            lastError = 0;
            return;
        }

        // ------------ start PD controller -------------

        double error = goalX - curID.getTx();
        double pTerm = error * kP;

        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        turretMotor.setPower(power);
        lastError = error;
    }


//    public void update(AprilTagDetection curID) {
//        double deltaTime = timer.seconds();
//        timer.reset();
//
//        if (curID == null) {
//            turretMotor.setPower(0);
//            lastError = 0;
//            return;
//        }
//
//        // ------------ start PD controller -------------
//
//        double error = goalX - curID.ftcPose.bearing;
//        double pTerm = error * kP;
//
//        double dTerm = 0;
//        if (deltaTime > 0) {
//            dTerm = ((error - lastError) / deltaTime) * kD;
//        }
//
//        if (Math.abs(error) < angleTolerance) {
//            power = 0;
//        } else {
//            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
//        }
//
//        turretMotor.setPower(power);
//        lastError = error;
//    }

}