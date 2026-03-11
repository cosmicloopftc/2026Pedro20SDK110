package AutoPrograms; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Blue Close V1.0", group = "Examples")
public class AutoBlueClose extends OpMode {
    public int currentSlot = 1;
    public String[] balls = {"NONE", "NONE", "NONE"};

    private static boolean firstTime;
    private static boolean secondTime;
    private static boolean startRunningLimelight = false;

    public static HardwareMain robot = new HardwareMain();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose startPose = new Pose(26, 130.5, Math.toRadians(-36)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(58, 102, Math.toRadians(150));
    private final Pose pickup1Pose = new Pose(48, 84, Math.toRadians(180)); // Scoring Pose of our robot.
    private final Pose pickup1Pose2 = new Pose(19, 84, Math.toRadians(180));

    private final Pose pickup2Pose = new Pose(48, 57, Math.toRadians(180));
    private final Pose pickup2Pose2 = new Pose(8.5, 57, Math.toRadians(180));

    private final Pose parkPose = new Pose(27,84, Math.toRadians(90));
    private PathChain goToPickup1, goToPickup2, grabPickup1, scorePickup1, grabPickup2, scorePickup2, scorePreload, park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .setConstraints(Constants.pathConstraints)
                .build();

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        goToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .setConstraints(Constants.pathConstraints)
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1Pose2))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1Pose2.getHeading())
                .setConstraints(Constants.pathConstraints)
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose2, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose2.getHeading(), scorePose.getHeading())
                .setConstraints(Constants.pathConstraints)
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        goToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .setTangentHeadingInterpolation()
                .setConstraints(Constants.pathConstraints)
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2Pose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2Pose2.getHeading())
                .setConstraints(Constants.pathConstraints)
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath( new BezierCurve(pickup2Pose2, new Pose(32.530, 55.939), scorePose))
                .setLinearHeadingInterpolation(pickup2Pose2.getHeading(), scorePose.getHeading())
                .setConstraints(Constants.pathConstraints)
                .build();

        park = follower.pathBuilder()
                .addPath( new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(pickup2Pose2.getHeading(), scorePose.getHeading())
                .setConstraints(Constants.pathConstraints)
                .build();
    }

    public void autonomousPathUpdate() {
        int time = 2500;
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    robot.spindexerShootingSlot1();
                    robot.hoodServo.setPosition(0.47);
                    robot.shooterVELO(-157);
//                    telemetry.addData("Flywheel speed", robot.leftShooterMotor.getVelocity());
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePreload, true);
                    if(pathTimer.getElapsedTime() > 150){
                        startRunningLimelight = true;
                    }
                    if(pathTimer.getElapsedTime() > 2000) {
                        robot.kickerUP();
//                        robot.intakeServoIN();
                        pathTimer.resetTimer();
                        setPathState(1);
                    }
//                    telemetry.addData("Elapsed time", pathTimer.getElapsedTime());

                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    time = 750;
                    if((pathTimer.getElapsedTime() < 750)){
                        robot.intakeSTOP();
                        robot.kickerUP();
                    }else if(pathTimer.getElapsedTime() < time+250) {
                        robot.kickerDOWN();
                    }
                    else if(pathTimer.getElapsedTime() < time+1000) {
                        robot.spindexerShootingSlot2();
                    }else if(pathTimer.getElapsedTime() < time+1750) {
                        robot.kickerUP();
                    }
                    else if(pathTimer.getElapsedTime() < time+2000) {
                        robot.kickerDOWN();
                    }
                    else if(pathTimer.getElapsedTime() < time+2750) {
                        robot.spindexerShootingSlot3();
                    }
                    else if(pathTimer.getElapsedTime() < time+3250) {
                        robot.kickerUP();
                    }
                    else if(pathTimer.getElapsedTime() < time+4000) {
                        robot.kickerDOWN();
                        robot.spindexerIntakeSlot1();
//                        robot.shooterOFF();
                        if(firstTime) {
                            startRunningLimelight = false;
                            setPathState(2);
//                            firstTime = false;
                        }else if(secondTime){
                            startRunningLimelight = false;
                            setPathState(5);
                        }else{
                            setPathState(12);
                        }
                    }
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    follower.followPath(goToPickup1, true);
                    if(follower.getPose().getY() >= pickup1Pose.getY()-0.5 && follower.getPose().getY() <= pickup1Pose.getY()+0.5 &&
                            follower.getPose().getY() >= pickup1Pose.getX()-0.5 && follower.getPose().getY() <= pickup1Pose.getX()+0.5){
                        setPathState(2);
                    }

                }
                break;
            case 2:
                telemetry.addData("balls[]", Arrays.toString(balls));

                robot.kickerDOWN();
                robot.intakeIN();
                //Intake code
            {
                double spindexerServoPosition = robot.getSpindexerServoPosition();

                if (spindexerServoPosition > 0.02 && spindexerServoPosition < 0.07 && robot.spindexerServo.getPosition() == 0){
                    currentSlot = 1;
                }
                else if (spindexerServoPosition > 0.37 && spindexerServoPosition < 0.4 && robot.spindexerServo.getPosition() == 0.37){
                    currentSlot = 2;
                }
                else if (spindexerServoPosition > 0.7 && spindexerServoPosition < 0.75 && robot.spindexerServo.getPosition() == 0.74){
                    currentSlot = 3;
                }

                String ball = ballDetected();
                telemetry.addData("balls", ball);

                if (currentSlot == 1 && balls[0].equals("NONE")) {
                    robot.spindexerIntakeSlot1();
                }

                if (!(ball.equals("NONE"))) {
                    telemetry.addLine("ACCESSED");
                    balls[currentSlot - 1] = ball;
                    if (currentSlot == 1) {
                        robot.spindexerIntakeSlot2();
                    } else if (currentSlot == 2) {
                        robot.spindexerIntakeSlot3();
                    } else {
                        robot.intakeOUT();
                        robot.spindexerShootingSlot1(); // TODO: switch to shooting state? intake out? stop intake?
                    }
                }
            }
            if (!follower.isBusy()) {
                follower.followPath(grabPickup1, 0.5,true);
                if(pathTimer.getElapsedTime() > 3000) {
                    setPathState(3);
                }
            }
            break;
            case 3:
                if (!follower.isBusy()) {
                    robot.spindexerShootingSlot1();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup1, true);

                    if(pathTimer.getElapsedTime() > 3500) {
//                        robot.hoodServo.setPosition(0.46);
//                        robot.intakeServoSTOP();
                        robot.kickerUP();
                        firstTime = false;
                        startRunningLimelight = true;
                        setPathState(1);
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 2000) {
                    /* Score Sample */
                    time = 1000;
                    if((pathTimer.getElapsedTime() < 1000)){
                        robot.intakeOUT();
//                        robot.intakeServoIN();
                    }else if(pathTimer.getElapsedTime() < time+250) {
                        robot.kickerDOWN();
                    }
                    else if(pathTimer.getElapsedTime() < time+1000) {
                        robot.spindexerShootingSlot2();
                    }else if(pathTimer.getElapsedTime() < time+1750) {
                        robot.kickerUP();
                    }
                    else if(pathTimer.getElapsedTime() < time+2250) {
                        robot.kickerDOWN();
                    }
                    else if(pathTimer.getElapsedTime() < time+2750) {
                        robot.spindexerShootingSlot3();
                    }
                    else if(pathTimer.getElapsedTime() < time+3250) {
                        robot.kickerUP();
                    }
                    else if(pathTimer.getElapsedTime() < time+3750) {
                        robot.kickerDOWN();
                        robot.spindexerIntakeSlot1();
//                        robot.intakeServoSTOP();
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goToPickup2, true);
                    setPathState(6);
                }
                break;
            case 6:
                //Intake code, put outside the !follower.isBusy() statement
            {
                double spindexerServoPosition = robot.getSpindexerServoPosition();

                if (spindexerServoPosition > 0.02 && spindexerServoPosition < 0.07 && robot.spindexerServo.getPosition() == 0){
                    currentSlot = 1;
                }
                else if (spindexerServoPosition > 0.37 && spindexerServoPosition < 0.4 && robot.spindexerServo.getPosition() == 0.37){
                    currentSlot = 2;
                }
                else if (spindexerServoPosition > 0.7 && spindexerServoPosition < 0.75 && robot.spindexerServo.getPosition() == 0.74){
                    currentSlot = 3;
                }

                String ball = ballDetected();

                if (currentSlot == 1 && balls[0].equals("NONE")) {
                    robot.spindexerIntakeSlot1();
                }

                if (!(ball.equals("NONE"))) {
                    balls[currentSlot - 1] = ball;
                    if (currentSlot == 1) {
                        robot.spindexerIntakeSlot2();
                    } else if (currentSlot == 2) {
                        robot.spindexerIntakeSlot3();
                    } else {
                        robot.intakeOUT();
                        robot.spindexerShootingSlot1(); // TODO: switch to shooting state? intake out? stop intake?
                    }
                }
            }
            if (!follower.isBusy()) {
                robot.intakeIN();
//                    robot.intakeServoIN();
                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                follower.followPath(grabPickup2, 0.4, true);
                if(pathTimer.getElapsedTime() > 2750) {
                    setPathState(7);
                }
            }
            break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    secondTime = false;
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    if(pathTimer.getElapsedTime() > 2250){
                        robot.spindexerShootingSlot1();
                        if(pathTimer.getElapsedTime() > 2750) {
                            startRunningLimelight = true;
                            setPathState(1);
                        }
                    }
                }
                break;
            case 12:
                if(!follower.isBusy()){
                    follower.followPath(park);
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        //AprilTag Tracking
        LLResult result = robot.limelight.getLatestResult();
        if(result.getTx() != 0 && robot.turretMotor.getCurrentPosition() < 245 && robot.turretMotor.getCurrentPosition() > -263 && startRunningLimelight) {
            double tx = result.getTx();
            double min_command = 0.02;
            double Kp = -0.015;
            double heading_error = -tx;
            double steering_adjust = 0.0;
            if (Math.abs(heading_error) > 1.0) {
                if (heading_error < 0) {
                    steering_adjust = Kp * heading_error + min_command;
                } else {
                    steering_adjust = Kp * heading_error - min_command;
                }
            }
            robot.turretMotor.setPower(steering_adjust);
            telemetry.addData("Tx:", steering_adjust);
        }else{
            robot.turretMotor.setPower(0);
        }

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("Elapsed time", pathTimer.getElapsedTime());
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Tx:", result.getTx());
        telemetry.addData("Is running", robot.limelight.isRunning());
        telemetry.addData("Spindexer position", robot.getSpindexerPosition());
        telemetry.addData("Shooter speed", robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.update();

        AutoToTeleopData.pose = follower.getPose();
        AutoToTeleopData.SpindexerServoPos = robot.spindexerServo.getPosition();
        AutoToTeleopData.hoodServoPos = robot.hoodServo.getPosition();
        AutoToTeleopData.turretMotorPos = robot.turretMotor.getCurrentPosition();

        AutoToTeleopData.limeLightPipeline = 1;
        AutoToTeleopData.autoRan = true;
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        firstTime = true;
        secondTime = true;
        robot.init(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);


        robot.limelight.pipelineSwitch(1);
        //Set up bulk data reading
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Starting pose", follower.getPose());
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        robot.limelight.start();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }

    public static String ballDetected(){
        String detectedBall = "";
//        if (hue > 200 && hue < 250) {
//            detectedColor = "PURPLE BALL";
//        } else if (hue > 150 && hue < 180) {
//            detectedColor = "GREEN BALL";
//        } else if (hue < 150) {
//            detectedColor = "NONE";
//        }
        if (robot.leftColorPin0.getState()) {
            detectedBall = "PURPLE BALL";
        } else if (robot.leftColorPin1.getState()) {
            detectedBall = "GREEN BALL";
        } else if (robot.rightColorPin1.getState()) {
            detectedBall = "BALL";
        } else{
            detectedBall = "NONE";
        }
        return detectedBall;
    }
}

