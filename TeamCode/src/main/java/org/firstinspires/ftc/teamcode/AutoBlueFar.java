package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Blue Far", group = "Examples")
public class AutoBlueFar extends OpMode {
    private static boolean startRunningLimelight = false;

    public static HardwareMain robot = new HardwareMain();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose startPose = new Pose(64, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose pickup1Pose = new Pose(44, 35, Math.toRadians(180)); // Scoring Pose of our robot.
    private final Pose pickup1Pose2 = new Pose(13, 35, Math.toRadians(180));
    private final Pose scorePose = new Pose(61, 15, Math.toRadians(120));

    private final Pose pickup2Pose = new Pose(9, 24, Math.toRadians(270));
    private final Pose pickup2Pose2 = new Pose(9, 10, Math.toRadians(270));
    private PathChain goToPickup1, goToPickup2, grabPickup1, scorePickup1, grabPickup2, scorePickup2, scorePreload;

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
                .setConstraints(Constants.pathConstraints)
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2Pose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2Pose2.getHeading())
                .setConstraints(Constants.pathConstraints)
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose2, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose2.getHeading(), scorePose.getHeading())
                .setConstraints(Constants.pathConstraints)
                .build();
    }

    public void autonomousPathUpdate() {
        int time = 2500;
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    robot.spindexerPosition1();
                    robot.shooterON();
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePreload, true);
                    if(pathTimer.getElapsedTime() > 250){
                        startRunningLimelight = true;
                    }
                    if(pathTimer.getElapsedTime() > 1500) {
                        robot.transferUP();
                        setPathState(1);
                    }
//                    telemetry.addData("Elapsed time", pathTimer.getElapsedTime());
                    telemetry.addData("Elapsed time", pathTimer.getElapsedTime());
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    telemetry.addData("Elapsed time", pathTimer.getElapsedTime());
                    time = 1000;
                    if((pathTimer.getElapsedTime() < 1000)){

                    }else if(pathTimer.getElapsedTime() < time+250) {
                        robot.transferDOWN();
                    }
                    else if(pathTimer.getElapsedTime() < time+1000) {
                        robot.spindexerPosition2();
                    }else if(pathTimer.getElapsedTime() < time+1750) {
                        robot.transferUP();
                    }
                    else if(pathTimer.getElapsedTime() < time+2000) {
                        robot.transferDOWN();
                    }
                    else if(pathTimer.getElapsedTime() < time+2750) {
                        robot.spindexerPosition3();
                    }
                    else if(pathTimer.getElapsedTime() < time+3250) {
                        robot.transferUP();
                    }
                    else if(pathTimer.getElapsedTime() < time+3750) {
                        robot.transferDOWN();
                        robot.spindexerPosition1();
                        robot.shooterOFF();
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    robot.transferDOWN();
                    robot.intakeIN();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goToPickup1, true);
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup1, true);
                    robot.shooterON();
                    if(pathTimer.getElapsedTime() > 3500) {
                        robot.transferUP();
                        setPathState(4);
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 2000) {
                    /* Score Sample */
                    time = 1000;
                    if((pathTimer.getElapsedTime() < 1000)){

                    }else if(pathTimer.getElapsedTime() < time+250) {
                        robot.transferDOWN();
                    }
                    else if(pathTimer.getElapsedTime() < time+1000) {
                        robot.spindexerPosition2();
                    }else if(pathTimer.getElapsedTime() < time+1750) {
                        robot.transferUP();
                    }
                    else if(pathTimer.getElapsedTime() < time+2000) {
                        robot.transferDOWN();
                    }
                    else if(pathTimer.getElapsedTime() < time+2750) {
                        robot.spindexerPosition3();
                    }
                    else if(pathTimer.getElapsedTime() < time+3250) {
                        robot.transferUP();
                    }
                    else if(pathTimer.getElapsedTime() < time+3750) {
                        robot.transferDOWN();
                        robot.spindexerPosition1();
                        robot.shooterOFF();
                        setPathState(-1);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goToPickup2, true);
                    follower.followPath(grabPickup2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup2, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    time = 1000;
                    if((pathTimer.getElapsedTime() < 500)){

                    }else if(pathTimer.getElapsedTime() < time+250) {
                        robot.transferDOWN();
                    }
                    else if(pathTimer.getElapsedTime() < time+500) {
                        robot.spindexerPosition2();
                    }else if(pathTimer.getElapsedTime() < time+750) {
                        robot.transferUP();
                    }
                    else if(pathTimer.getElapsedTime() < time+1250) {
                        robot.transferDOWN();
                    }
                    else if(pathTimer.getElapsedTime() < time+2000) {
                        robot.spindexerPosition3();
                    }
                    else if(pathTimer.getElapsedTime() < time+2500) {
                        robot.transferUP();
                    }
                    else if(pathTimer.getElapsedTime() < time+3000) {
                        robot.transferDOWN();
                        robot.shooterOFF();

                    }else if(pathTimer.getElapsedTime() < time+3500){
                        robot.spindexerPosition1();
                        time = 2500;
                        setPathState(-1);
                    }
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
        if(result.getTx() != 0 && robot.turretMotor.getCurrentPosition() < 300 && robot.turretMotor.getCurrentPosition() > -300 && startRunningLimelight) {
            double tx = result.getTx() ;
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
//            if(result.getTx() != 0 && robot.turretMotor.getCurrentPosition() >= 300 && (Kp * heading_error + min_command < 0 || Kp * heading_error - min_command < 0)){
//                double steering_adjust = 0.0;
//                if (Math.abs(heading_error) > 1.0) {
//                    if (heading_error < 0) {
//                        steering_adjust = Kp * heading_error + min_command;
//                    } else {
//                        steering_adjust = Kp * heading_error - min_command;
//                    }
//                }
//            }

            robot.turretMotor.setTargetPosition(0);
            robot.turretMotor.setPower(0);
        }

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Tx:", result.getTx());
        telemetry.addData("Is running", robot.limelight.isRunning());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
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
}
