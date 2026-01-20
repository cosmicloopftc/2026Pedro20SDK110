package org.firstinspires.ftc.teamcode;
import com.pedropathing.geometry.Pose;

/**
 In TeleOp,

 in init():
 WallDetector wallDetector = new WallDetector();



 in loop():

 // ---- Compute velocity ----
 double dx = p.x - lastPose.x;
 double dy = p.y - lastPose.y;
 double velX = dx / dt;
 double velY = dy / dt;

 double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

 // ---- Wall corrections (match-safe & invisible) ----
 if (wallDetector.frontWall(y, velX))
    follower.setPose(new Pose(FieldConstants.FRONT_WALL_X, p.y, blendHeading(p.heading, imuHeading)));

 if (wallDetector.backWall(y, velX))
    follower.setPose(new Pose(FieldConstants.BACK_WALL_X, p.y, blendHeading(p.heading, imuHeading)));

 if (wallDetector.leftWall(x, velY))
    follower.setPose(new Pose(p.x, FieldConstants.LEFT_WALL_Y, blendHeading(p.heading, imuHeading)));

 if (wallDetector.rightWall(x, velY))
    follower.setPose(new Pose(p.x, FieldConstants.RIGHT_WALL_Y, blendHeading(p.heading, imuHeading)));

 lastPose = follower.getPose();
 */


public class WallDetector {
    // ---- Tunables ----
    private static final double MIN_COMMAND = 0.25;
    private static final double MAX_VELOCITY = 0.6;   // inches/sec
    private static final long CONFIRM_TIME_MS = 250;
    private static final long COOLDOWN_MS = 1200;

    // ---- State ----
    private long hitStartTime = -1;
    private long lastTriggerTime = 0;

    // -------- PUBLIC API --------

    public boolean frontWall(
            double forwardCmd,
            double velocity,
            Pose pose
    ) {
        return detect(
                forwardCmd > MIN_COMMAND,
                velocity,
                facingFront(pose.getHeading())
        );
    }

    public boolean backWall(
            double forwardCmd,
            double velocity,
            Pose pose
    ) {
        return detect(
                forwardCmd < -MIN_COMMAND,
                velocity,
                facingBack(pose.getHeading())
        );
    }

    public boolean leftWall(
            double strafeCmd,
            double velocity,
            Pose pose
    ) {
        return detect(
                strafeCmd < -MIN_COMMAND,
                velocity,
                facingLeft(pose.getHeading())
        );
    }

    public boolean rightWall(
            double strafeCmd,
            double velocity,
            Pose pose
    ) {
        return detect(
                strafeCmd > MIN_COMMAND,
                velocity,
                facingRight(pose.getHeading())
        );
    }

    // -------- CORE LOGIC --------

    private boolean detect(
            boolean commandingTowardWall,
            double velocity,
            boolean headingValid
    ) {
        long now = System.currentTimeMillis();

        if (!headingValid) {
            hitStartTime = -1;
            return false;
        }

        if (now - lastTriggerTime < COOLDOWN_MS) {
            hitStartTime = -1;
            return false;
        }

        boolean stalled = Math.abs(velocity) < MAX_VELOCITY;

        if (commandingTowardWall && stalled) {

            if (hitStartTime < 0) hitStartTime = now;

            if (now - hitStartTime > CONFIRM_TIME_MS) {
                lastTriggerTime = now;
                hitStartTime = -1;
                return true;
            }
        } else {
            hitStartTime = -1;
        }

        return false;
    }

    // -------- HEADING GATES --------

    private boolean facingFront(double heading) {
        return within(heading, 0) || within(heading, Math.PI);
    }

    private boolean facingBack(double heading) {
        return within(heading, Math.PI);
    }

    private boolean facingLeft(double heading) {
        return within(heading, Math.PI / 2);
    }

    private boolean facingRight(double heading) {
        return within(heading, -Math.PI / 2);
    }

    private boolean within(double heading, double target) {
        return Math.abs(angleWrap(heading - target)) < Math.toRadians(30);
    }

    private double angleWrap(double a) {
        while (a <= -Math.PI) a += 2 * Math.PI;
        while (a > Math.PI) a -= 2 * Math.PI;
        return a;
    }
}
