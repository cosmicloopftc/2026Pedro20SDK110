//package org.firstinspires.ftc.teamcode;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import Hardware.HardwareDrivetrain;
//import Hardware.HardwareNoDriveTrainRobot;
//
//@Configurable
//public class TestLimelight extends LinearOpMode {
//
//    HardwareNoDriveTrainRobot robot = new HardwareNoDriveTrainRobot();
//    HardwareDrivetrain drivetrain = new HardwareDrivetrain();
//
//    Limelight3A limelight = hardwareMap.get(Limelight3A .class, "limelight");
//    @Override
//    public void runOpMode() throws InterruptedException {
//        float Kp = -0.1f;
//        float min_command = 0.05f;
//
//
//        LLResult result = limelight.getLatestResult();
//        double tx = result.getTx();
//
//
//        float heading_error = (float) -tx;
//        float steering_adjust = 0.0f;
//        if (Math.abs(heading_error) > 1.0)
//        {
//            if (heading_error < 0)
//            {
//                steering_adjust = Kp*heading_error + min_command;
//            }
//            else
//            {
//                steering_adjust = Kp*heading_error - min_command;
//            }
//        }
//        robot.turretMotor.setPower(steering_adjust);
//    }
//
//    public static double estimateDistance(){
//        Limelight3A limelight = hardwareMap.get(Limelight3A .class, "limelight");
//        LLResult result = limelight.getLatestResult();
//        double targetOffsetAngle_Vertical = result.getTy();
//
//        // how many degrees back is your limelight rotated from perfectly vertical?
//        //TODO get this variable
//        double limelightMountAngleDegrees = 25.0;
//
//        // distance from the center of the Limelight lens to the floor
//        //TODO get this variable
//        double limelightLensHeightInches = 20.0;
//
//        // distance from the target to the floor
//        //TODO look this up
//        double goalHeightInches = 60.0;
//
//        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
//        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
//
//        //calculate distance
//        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
//    }
//}
//
