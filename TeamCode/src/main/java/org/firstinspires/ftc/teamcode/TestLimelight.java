package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Configurable
public class TestLimelight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A .class, "limelight");
        Thread tracking = new Thread(new AprilTagTracking(limelight));
        tracking.start();
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

    public static double getLaunchAngle(Limelight3A limelight){
        double x = getDistanceToGoal(limelight);
        double y = 38.75-16;
        double g = 9.8;
        //The equation for surface speed, temporary calculation for muzzle velocity of ball
        double v = 0.5 * Math.PI * 0.096 * 6000;

        //This is a common value in the equation which I assigned to a variable to cut down on the number of calculations the computer had to do
        double discriminant = Math.sqrt((v*v + g*g*x*x - 2*y*g*v*v)/(g*x));
        double theta1 = Math.atan((Math.pow(v, 4) + discriminant) / (2*x));
        double theta2 = Math.atan((Math.pow(v, 4) - discriminant) / (g*x));

        if(theta1 < 0 || theta2 < 0){
            return Math.max(theta1, theta2);
        }
        return Math.min(theta1, theta2);
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
