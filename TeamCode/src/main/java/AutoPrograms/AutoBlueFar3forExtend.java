package AutoPrograms; // make sure this aligns with class location

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

import java.util.Arrays;
import java.util.List;

@Configurable
@Autonomous(name = "test BlueFar3forExtend V1.1", group = "Examples")
public class AutoBlueFar3forExtend extends OpMode {

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;

    private static boolean firstTime;
    private static boolean secondTime;
    private static boolean startRunningLimelight = false;

    public int currentSlot = 1;
    public String[] balls = {"NONE", "NONE", "NONE"};

    public static HardwareMain robot = new HardwareMain();

    private static Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

//    public Pose startPose; // Start Pose of our robot.
//    public Pose pickup1Pose; // Scoring Pose of our robot.
//    public Pose pickup1Pose2;
//    public Pose scorePose;
//    public Pose parkPose;
//
//    public Pose pickup2Pose;
//    public Pose pickup2Pose2;

    public int autoLimelightPipeline = 1;          //pipeline to BLUE = 1 (RED = 0)

    public Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of our robot.
    public Pose pickup1Pose = new Pose(51.5, 35, Math.toRadians(180)); // Scoring Pose of our robot.
    public Pose pickup1Pose2 = new Pose(11.5, 35, Math.toRadians(180));
    public Pose scorePose = new Pose(56, 13, Math.toRadians(120));
    public Pose parkPose = new Pose(47,19, Math.toRadians(90));

    public Pose pickup2Pose = new Pose(9, 35, Math.toRadians(270));
    public Pose pickup2Pose2 = new Pose(9, 11, Math.toRadians(270));


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
//                .setVelocityConstraint(0.5)
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose2, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose2.getHeading(), scorePose.getHeading())
                .setConstraints(Constants.pathConstraints)
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .setConstraints(Constants.pathConstraints)
                .build();
    }

    public void autonomousPathUpdate() {
        int time = 2500;
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    robot.spindexerShootingSlot1();
                    robot.hoodServo.setPosition(0.72);
                    robot.shooterVELO(-208);
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
                        if(!secondTime){
                            robot.hoodServo.setPosition(0.7);
                        }
                    }
                    else if(pathTimer.getElapsedTime() < time+3250) {
                        robot.kickerUP();
                    }
                    else if(pathTimer.getElapsedTime() < time+4000) {
                        robot.kickerDOWN();
                        robot.spindexerIntakeSlot1();
//                        robot.shooterOFF();
                        if(firstTime) {
                            setPathState(2);
//                            firstTime = false;
                        }else if(secondTime){
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
                    if(follower.getPose().getY() >= pickup1Pose.getY()-1 && follower.getPose().getY() <= pickup1Pose.getY()+1 &&
                            follower.getPose().getY() >= pickup1Pose.getX()-1 && follower.getPose().getY() <= pickup1Pose.getX()+1){
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
                    robot.shooterVELO(-208);

                    if(pathTimer.getElapsedTime() > 3500) {
//                        robot.hoodServo.setPosition(0.46);
//                        robot.intakeServoSTOP();
                        robot.kickerUP();
                        firstTime = false;
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
                follower.followPath(grabPickup2, 0.45, true);
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
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
//        Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of our robot.Pose pickup1Pose = new Pose(51.5, 35, Math.toRadians(180)); // Scoring Pose of our robot.
//        Pose pickup1Pose2 = new Pose(11.5, 35, Math.toRadians(180));
//        Pose scorePose = new Pose(56, 13, Math.toRadians(120));
//        Pose parkPose = new Pose(47,19, Math.toRadians(90));
//
//        Pose pickup2Pose = new Pose(9, 35, Math.toRadians(270));
//        Pose pickup2Pose2 = new Pose(9, 11, Math.toRadians(270));


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        firstTime = true;
        secondTime = true;
        robot.init(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robot.kickerDOWN();

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
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        //AprilTag Tracking
        LLResult result = robot.limelight.getLatestResult();
        if(result.getTx() != 0 && robot.turretMotor.getCurrentPosition() < 300 && robot.turretMotor.getCurrentPosition() > -233 && startRunningLimelight) {
            double tx = result.getTx() - 3.5;
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
        telemetryM.addData("Elapsed time", pathTimer.getElapsedTime());
        telemetryM.addData("path state", pathState);
        telemetryM.addData("x", follower.getPose().getX());
        telemetryM.addData("y", follower.getPose().getY());
        telemetryM.addData("heading", follower.getPose().getHeading());
        telemetryM.addData("Tx:", result.getTx());
        telemetryM.addData("Is running", robot.limelight.isRunning());
        telemetryM.addData("Spindexer position", robot.getSpindexerPosition());
        telemetryM.addData("Shooter speed", robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));
        telemetryM.update(telemetry);

        AutoToTeleopData.pose = follower.getPose();
        AutoToTeleopData.SpindexerServoPos = robot.spindexerServo.getPosition();
        AutoToTeleopData.hoodServoPos = robot.hoodServo.getPosition();
        AutoToTeleopData.turretMotorPos = robot.turretMotor.getCurrentPosition();

        AutoToTeleopData.limeLightPipeline = autoLimelightPipeline;
        AutoToTeleopData.autoRan = true;

        draw();
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


    public static void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void draw() {
        Drawing.drawDebug(follower);
    }



}