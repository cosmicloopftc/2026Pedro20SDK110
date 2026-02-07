package AutoPrograms;

import com.pedropathing.geometry.Pose;

public class AutoToTeleopData {

    public static Pose pose = new Pose(0, 0, 0);

    //TODO: set the value below at the starting position at Auto.
    //At end of auto, these values will be updated and then transfer to TeleOp
    public static double SpindexerServoPos = 0.0;
    public static double transferServoPos = 0.0;
    public static double hoodServoPos = 0.0;
    public static int turretMotorPos = 0;
    public static int limeLightPipeline = 0;
    public static boolean autoRan = false;


    /**
     * At end of Auto, put in these codes to save date to this class:
     AutoToTeleopData.pose = follower.getPose();
     AutoToTeleopData.SpindexerServoPos = robot.SpindexerServo.getPosition();
     AutoToTeleopData.transferServoPos = robot.transferServo.getPosition();
     AutoToTeleopData.hoodServoPos = robot.hoodServo.getPosition();
     AutoToTeleopData.turretMotorPos = robot.turretMotor.getPosition();
     AutoToTeleopData.imuHeading = robot.imu.getRobotYawPitchRollAngles()
                                    .getYaw(AngleUnit.DEGREES);
     AutoToTeleopData.autoRan = true;

     ----------------------------------------------------
     In TeleOp, under init(), insert these in:
     // Load Auto Data
     if (AutoToTeleopData.autoRan) {
        follower.setPose(AutoToTeleopData.pose);
        robot.SpindexerServo.setPosition(AutoToTeleopData.SpindexerServoPos);
        robot.transferServo.setPosition(AutoToTeleopData.transferServoPos);
        robot.hoodServo.setTargetPosition(AutoToTeleopData.hoodServoPos);
        robot.turretMotor.setTargetPosition(AutoToTeleopData.turretMotorPos);
     } else {
        follower.setPose(new Pose(0, 0, 0));
     }

     // IMU heading correction
     Pose p = follower.getPose();
     double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
     follower.setPose(new Pose(p.x, p.y, imuHeading));

     lastPose = follower.getPose();




     //Wall Detection and reset position, under loop()
     if (wallDetector.frontWallHit(commandedForward, velocity)) {
        Pose p = follower.getPose();
        follower.setPose(new Pose(
        FieldConstants.FRONT_WALL_X,
        p.y,
        imuHeading
        ));
     }



     */



}
