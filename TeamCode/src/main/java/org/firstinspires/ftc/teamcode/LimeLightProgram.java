package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@Configurable
@TeleOp
@Configurable
public class LimeLightProgram extends OpMode {
    private TelemetryManager telemetryM;
    @Override
    public void init() {


    }

    public void init_loop() {

    }

    @Override
    public void start() {


    }
    @Override
    public void loop() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        Limelight3A limelight = hardwareMap.get(Limelight3A .class, "limelight");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shootingMotor");

        if(gamepad1.a){
//        Thread tracking = new Thread(new AprilTagTracking(limelight));
//        tracking.start();
          telemetryM.debug("Angle: " + getLaunchAngle());
          telemetryM.update();
          shoot(shooter,true);
        }else{
            shoot(shooter,false);
        }
    }

    public static double getDistanceToGoal(Limelight3A limelight){
        LLResult result = limelight.getLatestResult();
        double targetOffsetAngle_Vertical = result.getTy();

        // how many degrees back is your limelight rotated from perfectly vertical?
        //TODO get this variable
        double limelightMountAngleDegrees = 25.0;

        // distance from the center of the Limelight lens to the floor
        //TODO get this variable
        double limelightLensHeightInches = 20.0;

        // distance from the target to the floor
        double goalHeightInches = 38.75;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

//    public static double getLaunchAngle(Limelight3A limelight){
    public static double getLaunchAngle(){
        double x = 74; //getDistanceToGoal(limelight);
        double y = 38.75-16.5;
        double g = 9.8;
        //The equation for surface speed, temporary calculation for muzzle velocity of ball
        double v = 0.5 * Math.PI * 0.096 * (6000 * 0.8);

        //This is a common value in the equation which I assigned to a variable to cut down on the number of calculations the computer had to do
        double c = (g*x*x)/(2*v*v);
        double discriminant = Math.sqrt((x*x - 4*c*c + 4*y*c)/(g*x));
        double theta1 = Math.atan((-x + discriminant) / (2*c));
        double theta2 =  Math.atan((-x - discriminant) / (2*c));

        return Math.min(theta1, theta2);
    }
    public static void shoot(DcMotorEx shooter, boolean startStop){
        if(startStop) {
            shooter.setPower(1);
        }else{
            shooter.setPower(0);
        }
    }
}
class AprilTagTracking implements Runnable {
    private Limelight3A limelight;

    AprilTagTracking(Limelight3A limelight) {
        this.limelight = limelight;
    }
    public void run(){
        while(true) {
            float Kp = -0.1f;
            float min_command = 0.05f;

            LLResult result = limelight.getLatestResult();
            double tx = result.getTx();

            float heading_error = (float) -tx;
            float steering_adjust = 0.0f;
            if (Math.abs(heading_error) > 1.0) {
                if (heading_error < 0) {
                    steering_adjust = Kp * heading_error + min_command;
                } else {
                    steering_adjust = Kp * heading_error - min_command;
                }
            }
            //robot.turretMotor.setPower(steering_adjust);
        }
    }
}
