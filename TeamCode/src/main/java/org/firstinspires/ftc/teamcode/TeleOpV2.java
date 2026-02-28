package org.firstinspires.ftc.teamcode;
//TODO:  need to add changes Dominic made on 1/22/2026 Wed-Thurs (master_endJan18LATEST_DCP branch)
//             in HardwareMain, and in TeleOpV2.java


import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HardwareDrivetrainNOTusingPedroPath;
import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**1/18/2026  correct manual move control (used PedroPathing teleOp method)
 * identify code to slow down manual move (slowModeMultiplier, slowMode) ; auto path (automatedDrive)
 *
 */

@Configurable
@TeleOp(name = "TeleOpV2 v1")
public class TeleOpV2 extends OpMode {
    public static double distance;
    int autoPipeline;
    public static boolean shooterOn = false;
    public static int currentSpeed = -210;

    ArrayList<Double> shootingResults = new ArrayList<Double>();
    boolean endGameRumble20secondsLeftOnce = true;
    boolean endGameRumble10secondsLeftOnce = true;
    Gamepad.RumbleEffect customRumbleEffect;

    private ElapsedTime runtime = new ElapsedTime();
//TODO:  edit END_GAME_TIME
    private final static double END_GAME_TIME = 30 - 20;
    private ElapsedTime loopCycleTime = new ElapsedTime();


    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.2;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timerLimelight = new ElapsedTime();
    private String shootingMode = "none";
    private String intakeMode = "normal";
    public static String detectedBall = "";
    final float[] right_hsvValues = new float[3];
    final float[] left_hsvValues = new float[3];
    final float[] back_hsvValues = new float[3];
    public String[] balls = {"NONE", "NONE", "NONE"};
    public int currentSlot = 1;

    public double shooterSpeed;

    public static HardwareMain robot = new HardwareMain();
    HardwareDrivetrainNOTusingPedroPath robotDrivetrain = new HardwareDrivetrainNOTusingPedroPath();
    enum State{
        INTAKE,
        SPINDEXER_STATE_ACTION, SHOOT
    }
    State state = State.INTAKE;
    @IgnoreConfigurable
    static PoseHistory poseHistory;



    @Override
    public void init() {
        robot.init(hardwareMap);
        robotDrivetrain.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        poseHistory = follower.getPoseHistory();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        //Set up bulk data reading
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        double imuHeading = robot.imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);

//        if (AutoToTeleopData.autoRan) {
//            follower.setPose(AutoToTeleopData.pose);
//            robot.spindexerServo.setPosition(AutoToTeleopData.SpindexerServoPos);
//            robot.transferServo.setPosition(AutoToTeleopData.transferServoPos);
//            robot.hoodServo.setPosition(AutoToTeleopData.hoodServoPos);
//            robot.turretMotor.setTargetPosition(AutoToTeleopData.turretMotorPos);
//            robot.limelight.pipelineSwitch(AutoToTeleopData.limeLightPipeline);
//        } else {
            follower.setPose(new Pose(0, 0, 0));
 //       }

        //Stores pipeline from auto
        autoPipeline = robot.limelight.getLatestResult().getPipelineIndex();

        //This has been moved to the HardwareMain class
//        robot.rightBallColorSensor.setGain(4.5F);
//        robot.leftBallColorSensor.setGain(4.5F);
//        robot.backBallColorSensor.setGain(4.5F);

    }

    public void init_loop() {
//        robot.limelight.pipelineSwitch(2);
        robot.LEDrightGreen.setMode(DigitalChannel.Mode.OUTPUT);
        robot.LEDrightRed.setMode(DigitalChannel.Mode.OUTPUT);
        robot.LEDleftGreen.setMode(DigitalChannel.Mode.OUTPUT);
        robot.LEDleftRed.setMode(DigitalChannel.Mode.OUTPUT);
        robot.LEDrightGreen.setState(true);
        robot.LEDleftGreen.setState(true);
        robot.LEDrightRed.setState(true);
        robot.LEDleftRed.setState(true);
        if(gamepad2.left_bumper){
            robot.limelight.pipelineSwitch(0);
//            autoPipeline = 0;
            telemetry.addLine("Red pipeline initialized");
        }else if(gamepad2.right_bumper){
            robot.limelight.pipelineSwitch(1);
//            autoPipeline = 1;
            telemetry.addLine("Blue pipeline initialized");
        }
        telemetry.addData("AutoPipeline", autoPipeline);
        telemetry.update();
        autoPipeline = robot.limelight.getLatestResult().getPipelineIndex();
    }

    @Override
    public void start() {
        resetRuntime();

        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        follower.update();
        robot.limelight.pipelineSwitch(3);
        robot.limelight.start();
        robot.LEDrightGreen.setState(false);
        robot.LEDleftGreen.setState(false);
        robot.LEDrightRed.setState(false);
        robot.LEDleftRed.setState(false);
    }
    @Override
    public void loop() {
        loopCycleTime.reset();
        telemetryM.addData("RunTime: ", (double) Math.round(getRuntime()* 10) / 10);

        distance = HardwareMain.getDistanceToGoal();
        robot.hoodOutFar();
        //AprilTagTracking
        LLResult result = robot.limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
        telemetry.addData("Tx", result.getTx());
        telemetry.addData("Is empty", aprilTags.isEmpty());
        if(!aprilTags.isEmpty()) {
            telemetry.addData("Id", aprilTags.get(0).getFiducialId());
        }
        telemetry.addData("Auto pipeline", autoPipeline);
        if(!aprilTags.isEmpty() && aprilTags.get(0).getFiducialId() == 20 && autoPipeline == 1){
            telemetry.addLine("ACCESSED");
            if(getDistanceToGoal() > 80) {
                robot.updateLimelight(-3.5);
            }else{
                robot.updateLimelight(0);
            }
        }else if(!aprilTags.isEmpty() && aprilTags.get(0).getFiducialId() == 24 && autoPipeline == 0){
            if(getDistanceToGoal() > 80) {
                robot.updateLimelight(2.75);
            }else{
                robot.updateLimelight(0);
            }
        }else{
            robot.turretMotor.setPower(0);
        }

        //close zone 163; far zone: 205
        shooterSpeed = robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES);
        if (shooterSpeed >= 195 && shooterSpeed <= 215 && !aprilTags.isEmpty()){
            //detect AprilTag AND shooterSpeed is at HIGH speed target: GREEN, GREEN (both tag and high speed)
            robot.LEDrightGreen.setState(true);
            robot.LEDleftGreen.setState(true);
            robot.LEDrightRed.setState(false);
            robot.LEDleftRed.setState(false);
        }
        else if (shooterSpeed >= 155 && shooterSpeed <= 175 && !aprilTags.isEmpty()) {
            //detect AprilTag AND shooterSpeed is at LOW speed target: GREEN, RED (both tag and low speed)
            robot.LEDrightGreen.setState(false);
            robot.LEDleftGreen.setState(true);
            robot.LEDrightRed.setState(true);
            robot.LEDleftRed.setState(false);
        }
        else if (!aprilTags.isEmpty()){
            //detect AprilTag only: ORANGE, ORANGE
            robot.LEDrightGreen.setState(true);
            robot.LEDleftGreen.setState(true);
            robot.LEDrightRed.setState(true);
            robot.LEDleftRed.setState(true);
        }
        else{
            //NO AprilTag AND not at any speed target: RED, RED  (don't bother shooting.)
            robot.LEDrightGreen.setState(false);
            robot.LEDleftGreen.setState(false);
            robot.LEDrightRed.setState(true);
            robot.LEDleftRed.setState(true);
        }





//        if (robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES) >= 150 && robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES) <= 220 && !aprilTags.isEmpty()){
//            robot.LEDrightGreen.setState(true);
//            robot.LEDleftGreen.setState(true);
//            robot.LEDrightRed.setState(false);
//            robot.LEDleftRed.setState(false);
//        }
//        else if(robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES) >= 150 && robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES) <= 220){
//            robot.LEDrightGreen.setState(true);
//            robot.LEDleftGreen.setState(true);
//            robot.LEDrightRed.setState(true);
//            robot.LEDleftRed.setState(true);
//        }
//        else if(robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES) >= 150 && robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES) <= 220){
//            robot.LEDrightGreen.setState(true);
//            robot.LEDleftGreen.setState(true);
//            robot.LEDrightRed.setState(true);
//            robot.LEDleftRed.setState(true);
//        }
//        else{
//            robot.LEDrightGreen.setState(false);
//            robot.LEDleftGreen.setState(false);
//            robot.LEDrightRed.setState(true);
//            robot.LEDleftRed.setState(true);
//        }

//        if (gamepad2.a){
//            robot.transferDOWN();
//        }
//        else if (gamepad2.b){
//            robot.transferUP();
//        }

        switch (state) {
            case INTAKE:

                robot.transferSTOP();
                if (currentSlot == 1){
                    robot.spindexerIntakeSlot1();
                }
                else if (currentSlot == 2){
                    robot.spindexerIntakeSlot2();
                }
                else if (currentSlot == 3 && balls[2].equals("NONE")){
                    robot.spindexerIntakeSlot3();
                }

                String ball = ballDetected();

                if (!(ball.equals("NONE"))){
                    balls[currentSlot - 1] = ball;
                    if (currentSlot < 3) {
                        currentSlot += 1;
                    }
                    else{
                        robot.spindexerShootingSlot1(); // TODO: switch to shooting state? intake out? stop intake?
                    }
                }

                if (gamepad1.dpad_up){
                    intakeMode = "normal";
                    robot.intakeIN();
                }
                else if (gamepad1.dpad_down){
                    intakeMode = "normal";
                    robot.intakeOUT();
                }

                else if (gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.left_bumper) {
                    intakeMode = "normal";
                    robot.intakeSTOP();
                }
                else if (gamepad1.a){
                    state = State.SHOOT;
                }

                if (gamepad2.b && (((!aprilTags.isEmpty() && ((aprilTags.get(0).getFiducialId() == 20 && autoPipeline == 1) || (aprilTags.get(0).getFiducialId() == 24 && autoPipeline == 0)))) || !(robot.limelight.isConnected() && robot.limelight.isRunning()))){
                    //robot.auto3Shoot();
                    shootingMode = "all3";
                    timer.reset();
                    state = State.SHOOT;
                }
                else if (gamepad2.right_bumper){
                    shootingMode = "manual";
                    state = State.SHOOT;
                }

                break;
            case SHOOT:
                // Old version of all 3:
//                if (shootingMode.equals("all3")){
//                    if (timer.milliseconds() > 0 && timer.milliseconds() < 250){
//                        robot.spindexerPosition1();
//                        robot.transferUP();
//                        shootingResults.add(robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));
//                    }
//                    else if(timer.milliseconds() >= 250 && timer.milliseconds() < 450){
//                        robot.spindexerPosition1();
//                        robot.transferDOWN();
//                    }
//                    else if(timer.milliseconds() >= 450 && timer.milliseconds() < 900){
//                        robot.spindexerPosition2();
//                    }
//                    else if(timer.milliseconds() >= 900 && timer.milliseconds() < 1150){
//                        robot.spindexerPosition2();
//                        robot.transferUP();
//                        shootingResults.add(robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));
//                    }
//                    else if(timer.milliseconds() >= 1150 && timer.milliseconds() < 1350){
//                        robot.spindexerPosition2();
//                        robot.transferDOWN();
//                    }
//                    else if(timer.milliseconds() >= 1350 && timer.milliseconds() < 1850){
//                        robot.spindexerPosition3();
//                    }
//                    else if(timer.milliseconds() >= 1850 && timer.milliseconds() < 2150){
//                        robot.spindexerPosition3();
//                        robot.transferUP();
//                        shootingResults.add(robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));
//                    }
//                    else if(timer.milliseconds() >= 2150 && timer.milliseconds() < 2350){
//                        robot.spindexerPosition3();
//                        robot.transferDOWN();
//                    }
//                    else if(timer.milliseconds() > 2350){
//                        robot.spindexerPosition1();
//                        shootingMode = "none";
//                    }
//                }
//                else if (shootingMode.equals("manual")){
//                    if (gamepad2.right_bumper && (((!aprilTags.isEmpty() && ((aprilTags.get(0).getFiducialId() == 20 && autoPipeline == 1) || (aprilTags.get(0).getFiducialId() == 24 && autoPipeline == 0)))) || !(robot.limelight.isRunning() && robot.limelight.isConnected()))){
//                        robot.transferUP();
//                        shootingResults.add(robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));
//                        timer.reset();
//                    }
//                    else if(timer.milliseconds() > 200){
//                        robot.transferDOWN();
//                    }
//                    if (gamepad2.left_trigger >= 0.2){
//                        robot.spindexerPosition1();
//                    }
//                    else if (gamepad2.left_bumper){
//                        robot.spindexerPosition2();
//                    }
//                    else if(gamepad2.right_trigger >= 0.2){
//                        robot.spindexerPosition3();
//                    }
//                    if (gamepad1.dpad_up || gamepad2.x){
//                        shootingMode = "none";
//                    }
//                }
//                else if (shootingMode.equals("color")){
//                    robot.intakeServoIN();
//                    NormalizedRGBA rightColor = robot.rightBallColorSensor.getNormalizedColors();
//                    NormalizedRGBA leftColor = robot.leftBallColorSensor.getNormalizedColors();
//                    NormalizedRGBA backColor = robot.backBallColorSensor.getNormalizedColors();
//                    Color.colorToHSV(rightColor.toColor(), right_hsvValues);
//                    Color.colorToHSV(leftColor.toColor(), left_hsvValues);
//                    Color.colorToHSV(backColor.toColor(), back_hsvValues);
//
//                    if (gamepad2.right_trigger > 0.1 && (ColorBallDetected(right_hsvValues[0]).equals("GREEN") || ColorBallDetected(left_hsvValues[0]).equals("GREEN") || ColorBallDetected(back_hsvValues[0]).equals("GREEN"))){
//
//                    }
//                    else if (gamepad2.left_trigger > 0.2){
//
//                    }
//                }
                if (shootingMode.equals("none")){
                    state = State.INTAKE;
                }
                break;
        }
//        if (gamepad2.y){
//            robot.shooterON();
//        }
        if (gamepad2.dpad_up){
//            robot.hoodMID();
            //robot.shooterVELO(-210); // Far launch zone, overshoot
            //robot.shooterVELO(-200); // Far launch zone, almost over shoot
            robot.shooterON(); // Far launch zone, bounce out
            //robot.shooterVELO(-190); // Far launch zone, bounce out
            //robot.shooterVELO(-185); // Far launch zone, bounce out
            shooterOn = true;
        }
        else if (gamepad2.dpad_down){
            robot.hoodMID();
            //robot.shooterVELO(-167); // Near launch zone, equivalent to 0.54 power
            robot.shooterVELO(-160); // Near launch zone, equivalent to 0.52 power
            //robot.shooterVELO(-150); // Near launch zone, equivalent to 0.50 power
            //robot.shooterVELO(-145); // Near launch zone, equivalent to 0.48 power
            shooterOn = true;
        }
        else if (gamepad2.a){
            robot.shooterOFF();
            shooterOn = false;
        }else if(gamepad2.dpad_left){
            robot.hoodOutFar();
        }
        else if(gamepad2.dpad_right){
            shooterOn = true;
        }
        if(shooterOn){
            if(getDistanceToGoal() > 80 && result.isValid()) {
                currentSpeed = -208;
            }else if(result.isValid()){
                currentSpeed = -165;
            }
            if(result.isValid()){
                robot.shooterVELO(currentSpeed);
            }
            telemetry.addData("Target speed", currentSpeed);
            telemetry.addData("Result", result.isValid());
        }else{
            robot.shooterOFF();
        }
//        if(gamepad2.dpad_down){
//            robot.hoodIN();
//        }else if(gamepad2.dpad_left){
//            robot.hoodOutFar();
//        }
//        else if(gamepad2.dpad_right){
//            robot.hoodOutClose();
//        }
//        else if(gamepad2.dpad_up){
//            robot.hoodServo.setPosition(getLaunchAngle());
//        }
//        else if (gamepad1.ps){
//            robot.transferOFF();
//        }


        //Call this once per loop
        follower.update();
        telemetryM.update();
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }


        if ((runtime.seconds() > 100) && endGameRumble20secondsLeftOnce) {
            rumble();
            telemetry.addLine("20 SECONDS LEFT");
            endGameRumble20secondsLeftOnce = false;
        }

        //3 rumble for 8 seconds left for hanging
        if ((runtime.seconds() > 110) && endGameRumble10secondsLeftOnce) {
            rumble();
            telemetry.addLine("10 SECONDS");
            endGameRumble10secondsLeftOnce = false;
        }

//        //Automated PathFollowing                //TODO: This is where we can automate a path
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }
//        //Stop automated following if the follower is done //TODO: stop automate a path when 1) manually stopped or done with path
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }
//        //Slow Mode                               //TODO: slow down robot movement by multiplying a slowModeMultiplier = 0.2 (very slow)
//        if (gamepad1.rightBumperWasPressed()) {
//            slowMode = !slowMode;
//        }
//        //Optional way to change slow mode strength  //TODO: slow down robot movement by multiplying a
//        if (gamepad1.xWasPressed()) {
//            slowModeMultiplier += 0.25;
//        }
//        //Optional way to change slow mode strength
//        if (gamepad2.yWasPressed()) {
//            slowModeMultiplier -= 0.25;
//        }
        telemetry.addData("Is running and connected", robot.limelight.isRunning() && robot.limelight.isConnected());
        telemetry.addData("Hood position", robot.hoodServo.getPosition());
        telemetry.addData("Launch angle", HardwareMain.getLaunchAngle());
        telemetry.addData("Distance to goal", HardwareMain.getDistanceToGoal());
        telemetry.addData("Angle to goal from lightlight", result.getTy());
        telemetry.addData("Turret Motor Position:", robot.turretMotor.getCurrentPosition());
        telemetry.addData("Shooter velocity TPS:", robot.leftShooterMotor.getVelocity());
//        telemetry.addData("Spindexer position", robot.getSpindexerPosition());
//        telemetry.addData("Spindexer position2", robot.spindexerServo.getPosition() == 0.38);
//        String[] target = {"PURPLE BALL", "GREEN BALL", "PURPLE BALL"};
//        telemetry.addData("Current sorted shot", robot.getSortingPosition(target, new String[3], 1));


        telemetry.addData("rightShooter AngVel (deg/s?): ", robot.rightShooterMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("leftShooter  AngVel (deg/s?): ", robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        telemetry.addData("Left digital 0", robot.leftColorPin0.getState());
        telemetry.addData("Left digital 1", robot.leftColorPin1.getState());
        telemetry.addData("Right digital 0: 5 mm away", robot.rightColorPin0.getState());
        telemetry.addData("Right digital 1: 10 mm away", robot.rightColorPin1.getState());
        telemetry.addData("BALL = ", ballDetected());
        telemetry.addData("Slot 1 = ", balls[0]);
        telemetry.addData("Slot 2 = ", balls[1]);
        telemetry.addData("Slot 3 = ", balls[2]);

        telemetryM.addLine("");
        telemetryM.debug("loopCycleTime (ms): " + (double) Math.round(loopCycleTime.milliseconds() * 100) / 100);

        telemetryM.update(telemetry);
        telemetry.update();

//        telemetry.addLine()
//                .addData("Hue", "%.3f", left_hsvValues[0])
//                .addData("Left BALL = ", ColorBallDetected(left_hsvValues[0]));
//        telemetry.addLine()
//                .addData("Hue", "%.3f", back_hsvValues[0])
//                .addData("Back BALL = ", ColorBallDetected(back_hsvValues[0]));

    }

    @Override
    public void stop(){
        telemetry.addData("", shootingResults);
        telemetry.update();
    }



    public static double getDistanceToGoal(){
        LLResult result = robot.limelight.getLatestResult();
        double targetOffsetAngle_Vertical = result.getTy();
        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 19.48;  //32.39;

        // distance from the center of the Limelight lens to the floor
        //TODO get this variable
        double limelightLensHeightInches = 11.5;

        // distance from the target to the floor
        double goalHeightInches = 29.25;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    public static double getLaunchAngle(){
        double x = getDistanceToGoal()/39.37;
        double y = 27.25/39.37;
        double g = 9.8;

        double v = 7.3;

        //This is a common value in the equation which I assigned to a variable to cut down on the number of calculations the computer had to do
        double C = (g*x*x) / (2*v*v);
        double discriminant = Math.sqrt((x*x - 4*(-C)*(-C-y)));
        double theta1 = Math.atan((-x + discriminant) / (2*(-C)));
        double theta2 = Math.atan((-x - discriminant) / (2*(-C)));

//        return (1-(Math.min(theta1, theta2) * 180/Math.PI)/0.0356) - 0.01; //37 = 0
        return -0.0312086*(Math.min(theta1, theta2) * 180/Math.PI)+1.80914 -0.01;
    }


    public void rumble(){
        Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();
        gamepad1.runRumbleEffect(customRumbleEffect);
        gamepad2.runRumbleEffect(customRumbleEffect);
    }


    //TODO:  need to adjust the Hue value limit based on testing at different light setting.
    public static String ballDetected(){
//        if (hue > 200 && hue < 250) {
//            detectedColor = "PURPLE BALL";
//        } else if (hue > 150 && hue < 180) {
//            detectedColor = "GREEN BALL";
//        } else if (hue < 150) {
//            detectedColor = "NONE";
//        }
        if (robot.leftColorPin0.getState() == true) {
            detectedBall = "PURPLE BALL";
        } else if (robot.leftColorPin1.getState() == true) {
            detectedBall = "GREEN BALL";
        } else if (robot.rightColorPin1.getState() == true) {
            detectedBall = "BALL";
        } else{
            detectedBall = "NONE";
        }
        return detectedBall;
    }

}