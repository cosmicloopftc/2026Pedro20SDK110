package org.firstinspires.ftc.teamcode;

/** copy over from Marcus' branch 1/18/2026
 */

//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import android.sax.StartElementListener;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

//FTC Thunderbolts (S0acramento, CA) mentor program structure
//1:01:slider example adjusts the motor speed for the slide for it to reach a set Encoder value.
//OpMode structure: init(), init_loop(), start(), loop(), stop().  as of SDK 8.1, 1ms between call in loop()
//LinearOpMode structure: runOpMode(), waitForStart(), isStarted(), isStopRequested(), idle(), opModeIsActive(), opModeInInit()



public class TurretMechanism {
    private double correctedTurretAngleWhileMoving = 0;     //TODO: negative when aiming to right goal
    private double xVel_Robot = 0;

//TODO: this is for PID method 2--difficult to tune
//    private double accumulatedError = 0;
//    private double lastTime = 0;
//    private double kI = 0;
//    boolean firstDetectID = false;


    private double goalTurretAngleRelField = -14.0;


    private String PIDmethod;
    private DcMotorEx turretMotor;

    public static double kP = 0.024;            //pre 3/6/2025 scrimmage: kP = 0.023
    public static double kD = 0.00023;          //pre 3/6/2025 scrimmage: kD = 0.00023

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

    public double getPower() {
        return power;
    }
    public String getPIDmethod() {
        return PIDmethod;
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
//        }else{
//            firstDetectID = true;
        }

        //TODO: PID controller method 1
        // ------------ start PD controller -------------
        PIDmethod = "PD w/ feed forward controller ";
        double error = goalX - curID.getTx();
        double pTerm = error * kP;
        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }
        if (Math.abs(error) < angleTolerance){
            power = 0;
        } else {
            power = -Range.clip(
                    (0.04)*Math.signum(error) + pTerm + dTerm,
                    -MAX_POWER, MAX_POWER);
            //Math.signum(d)
            //1.0 (or 1.0f) if the number passed is greater than zero.
            //-1.0 (or -1.0f) if the number passed is less than zero.
            //0.0 (or 0.0f) if the number passed is equal to zero.
        }


        //TODO: PID controller method 2--does not work well, hard to tune
//        //------------ start PD controller with feed forward and time slope method
//        PIDmethod = "PD controller w/ feed forward, by time slope";
//        //P
//        double error = goalX - curID.getTx();
//        //I
//        accumulatedError += error;
//        //D slow down robot when oscillate too much; based on slope; assume robot travel in diagonal line
//        double slope = 0;
//        if (lastTime > 0) {
//            slope = (error - lastError)/(timer.milliseconds() - lastTime);
//        }
//        lastTime = timer.milliseconds();
//        lastError = error;
//        //motor power calculation, feed forward control, tanh is between -1 and 1
//        if (Math.abs(error) < angleTolerance) {
//            power = 0;
//        } else {
//            power = -Range.clip(
//                    (0.05)*Math.signum(error) +
//                    (0.95)*Math.tanh(kP*error + kI*accumulatedError + kD*slope),
//                    -MAX_POWER, MAX_POWER
//                    );
//        }

        turretMotor.setPower(power);
        lastError = error;
    }


    public void updateByPedroPath(double currentTurretAngleRelField_Deg, double goalTurretAngleRelField_Deg) {
        double deltaTime = timer.seconds();
        timer.reset();
//TODO: PID controller method 1
        // ------------ start PD controller -------------
        PIDmethod = "PD w/ feed forward controller ";
        double error = goalTurretAngleRelField_Deg - currentTurretAngleRelField_Deg;
        double pTerm = error * kP;
        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }
        if (Math.abs(error) < angleTolerance){
            power = 0;
        } else {
            power = -Range.clip(
                    (0.05)*Math.signum(error) + pTerm + dTerm,
                    -MAX_POWER, MAX_POWER);
            //Math.signum(d)
            //1.0 (or 1.0f) if the number passed is greater than zero.
            //-1.0 (or -1.0f) if the number passed is less than zero.
            //0.0 (or 0.0f) if the number passed is equal to zero.
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