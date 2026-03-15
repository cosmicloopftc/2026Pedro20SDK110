package org.firstinspires.ftc.teamcode;
//TODO:  need to add changes Dominic made on 1/22/2026 Wed-Thurs (master_endJan18LATEST_DCP branch)
//             in HardwareMain, and in TeleOpV2.java


//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.HardwareDrivetrainNOTusingPedroPath;
import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import AutoPrograms.AutoToTeleopData;
import AutoPrograms.AutoToTeleopData.*;



/**1/18/2026  correct manual move control (used PedroPathing teleOp method)
 * identify code to slow down manual move (slowModeMultiplier, slowMode) ; auto path (automatedDrive)
 */

@Configurable
@TeleOp(name = "BLUE TeleOpV2 v1.3", group = "A")
public class BLUE_TeleOpV2 extends OpMode {
    double robotImuHeadingDeg;

    private TurretMechanism turret = new TurretMechanism();

    int maxAllowTurretTick = 829, minAllowTurretTick = - -840, turretTickAt90Degree = 667;       //set limit of Turret position on robot
    double turretAngleRelativeToRobot_Deg, turretAngleRelativeToField_Deg;
    double GoalX = 15 , GoalY = 128;        // TODO: need to confirm with Dominic
    String turrentAimingMethod;

    public static double distance;
    int autoPipeline = 1;                    // TODO: set pipeline to blue =1 (red = 0)

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
    public Pose startingPose = new Pose(38,33, Math.toRadians(90));; //See ExampleAuto to understand how to use this

    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.2;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime bumpCorrectionTimer = new ElapsedTime();

    private String shootingMode = "none";
    private String intakeMode = "normal";
    public static String detectedBall = "";
    final float[] right_hsvValues = new float[3];
    final float[] left_hsvValues = new float[3];
    final float[] back_hsvValues = new float[3];
    public String[] balls = {"NONE", "NONE", "NONE"};
    public int currentSlot = 1;
    public boolean intakeSlotPosition = true;
    double Brushland_LeftHue;
    double Brushland_RightHue;
    String ball = "NONE";

    //Analog spindexer non-intaking position ranges
    public static double yaw;
    boolean executeOnce = true;

    public double shooterSpeed;

    public static HardwareMain robot = new HardwareMain();
    HardwareDrivetrainNOTusingPedroPath robotDrivetrain = new HardwareDrivetrainNOTusingPedroPath();
    public enum State{
        INTAKE,
        SPINDEXER_STATE_ACTION, SHOOT
    }
    public static State state = State.INTAKE;
    @IgnoreConfigurable
    static PoseHistory poseHistory;

    boolean colorIntakeToggle = false;


    @Override
    public void init() {
        robot.init(hardwareMap);
        robotDrivetrain.init(hardwareMap);
        turret.init(hardwareMap);

        if (AutoPrograms.AutoToTeleopData.autoRan) {
            startingPose = AutoToTeleopData.pose;
        }



        //TODO: need to adjust this starting point--for now it is at BLUE endgame box--robot and turret point straight forward.
        //TODO: set startingPose to start TeleOp--tx from Auto?
        //blue Endgame box = (105, 33, ...); //red Endgame box = (38, 33, ....)
        //TODO: need to confirm with Dominic:  BLUE goal ?? (14,128);   RED goal (129, 128)
        //For now, at start of TeleOp, set BLUE FAR robot to be at BLUE Endgame box = (38, 33, ....) and point up to goal Y direction
        //For now, at start of TeleOp, set BLUE NEAR robot to be at ??? and point up to goal Y direction
        //startingPose = new Pose(38,33, Math.toRadians(90));

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

        robotImuHeadingDeg = robot.imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES) ;
        telemetryM.addData("robotImuHeadingDeg (degrees) = ",  (double) Math.round(robotImuHeadingDeg) * 10 / 10);

//        if (AutoToTeleopData.autoRan) {
//            follower.setPose(AutoToTeleopData.pose);
//            robot.spindexerServo.setPosition(AutoToTeleopData.SpindexerServoPos);
//            robot.transferServo.setPosition(AutoToTeleopData.transferServoPos);
//            robot.hoodServo.setPosition(AutoToTeleopData.hoodServoPos);
//            robot.turretMotor.setTargetPosition(AutoToTeleopData.turretMotorPos);
//            robot.limelight.pipelineSwitch(AutoToTeleopData.limeLightPipeline);
//        } else {
        //follower.setPose(new Pose(0, 0, 0));
        follower.setPose(startingPose);
        //       }

        //Stores pipeline from auto
        //       autoPipeline = robot.limelight.getLatestResult().getPipelineIndex();
//        autoPipeline = 1;  // TODO: set pipeline to blue
        robot.limelight.pipelineSwitch(autoPipeline);

        //This has been moved to the HardwareMain class
//        robot.rightBallColorSensor.setGain(4.5F);
//        robot.leftBallColorSensor.setGain(4.5F);
//        robot.backBallColorSensor.setGain(4.5F);
        robot.statusRGB.setPosition(0.388);         //yellow = 0.388
    }

    public void init_loop() {
        telemetryM.addLine("Initialized all mechanisms");

//        robot.limelight.pipelineSwitch(2);
//        robot.LEDrightGreen.setMode(DigitalChannel.Mode.OUTPUT);
//        robot.LEDrightRed.setMode(DigitalChannel.Mode.OUTPUT);
//        robot.LEDleftGreen.setMode(DigitalChannel.Mode.OUTPUT);
//        robot.LEDleftRed.setMode(DigitalChannel.Mode.OUTPUT);
//        robot.LEDrightGreen.setState(true);
//        robot.LEDleftGreen.setState(true);
//        robot.LEDrightRed.setState(true);
//        robot.LEDleftRed.setState(true);
//        if(gamepad2.left_bumper){
//            robot.limelight.pipelineSwitch(0);
//// TODO: ask Dominic if the next line needs to be commented out because this has already been set by initial auto run at start of match
//            autoPipeline = 0;
//            telemetry.addLine("Red pipeline initialized");
//        }else if(gamepad2.right_bumper){
//            robot.limelight.pipelineSwitch(1);
//// TODO: ask Dominic if the next line needs to be commented out because this has already been set by initial auto run at start of match
//            autoPipeline = 1;
//            telemetry.addLine("Blue pipeline initialized");
//        }



        telemetryM.addData("AutoPipeline", autoPipeline);
        telemetryM.addData("Goal X", GoalX);
        telemetryM.addData("Goal Y", GoalY);
        telemetryM.addData("current Pose X",(double) Math.round(follower.getPose().getX()) * 10 / 10);
        telemetryM.addData("current Pose Y", (double) Math.round(follower.getPose().getY()) * 10 / 10);

        telemetryM.addLine("");

        double robotImuHeadingDeg = robot.imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);
        double robotPedroPathHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        turretTickAt90Degree = 667;       //set limit of Turret position on robot
        turretAngleRelativeToRobot_Deg = robot.turretMotor.getCurrentPosition()*90/turretTickAt90Degree;
        turretAngleRelativeToField_Deg =  robotPedroPathHeadingDeg - turretAngleRelativeToRobot_Deg;
        double goalTurretAngleRelField_Deg = Math.toDegrees(Math.atan((GoalY - follower.getPose().getY())/(GoalX - follower.getPose().getX())));
        telemetryM.addData("Cur turret motor encoder = ", (double) Math.round(robot.turretMotor.getCurrentPosition()));
        telemetryM.addData("Cur turretAngleRelativeToField_Deg = ", (double) Math.round(turretAngleRelativeToField_Deg) * 10 / 10);
        telemetryM.addData("Goal TurretAngRelField_Deg = ",  (double) Math.round(goalTurretAngleRelField_Deg) * 10 / 10);
        telemetryM.addData("Goal TurretAngRelField_Deg = ",  (double) Math.round(goalTurretAngleRelField_Deg) * 10 / 10);

        // TODO: set pipeline to blue =1 (red = 0)
        if (autoPipeline==1){
            telemetryM.addData("autoPipeline = BLUE = ", autoPipeline);
            telemetryM.addLine("Note: Not checking Limelight");
            robot.statusRGB.setPosition(0.48);         //yellow = 0.388; green =0.48, ready, done init
        }else if (autoPipeline==0) {
            telemetryM.addData("autoPipeline = RED = ", autoPipeline);
            telemetryM.addLine("Note: Not checking Limelight");
            robot.statusRGB.setPosition(0.48);         //yellow = 0.388; green =0.48, ready, done init
        }else{
            telemetryM.addData("ERROR in autoPipeline ", autoPipeline);
            telemetryM.addLine("Note: Not checking Limelight");
            robot.statusRGB.setPosition(0.28);         //yellow = 0.388; green =0.48, ready, done init; Red =error = 0.28
        }

        telemetryM.update(telemetry);

// TODO: ask Dominic if the next line needs to be commented out because this has already been set by initial auto run at start of match
        //      autoPipeline = robot.limelight.getLatestResult().getPipelineIndex();


    }

    @Override
    public void start() {
        //robot.imu.resetYaw();

        resetRuntime();
        turret.resetTimer();
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        follower.update();
        robot.limelight.start();
//        robot.limelight.pipelineSwitch(1);
//        robot.LEDrightGreen.setState(false);
//        robot.LEDleftGreen.setState(false);
//        robot.LEDrightRed.setState(false);
//        robot.LEDleftRed.setState(false);
        robot.statusRGB.setPosition(0);
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
        telemetryM.addData("Tx",  (double) Math.round(result.getTx() * 100 / 100));
        telemetryM.addData("Is empty", aprilTags.isEmpty());



        if (getRuntime() > 90) {
            ball = colorBallDetected();
        }
        else{
            ball = ballDetected();
        }

//        if(gamepad1.psWasPressed() && !colorIntakeToggle){
//            colorIntakeToggle = true;
//            ball = colorBallDetected();
//        }
//        else if(!gamepad1.psWasPressed() && colorIntakeToggle){
//            colorIntakeToggle = false;
//            ball = ballDetected();
//        }




        boolean leftColorPurple = robot.leftColorPin0.getState();
        boolean leftColorGreen = robot.leftColorPin1.getState();
        boolean rightColorPurple = robot.rightColorPin0.getState();
        boolean rightColorGreen = robot.rightColorPin1.getState();




        if(!aprilTags.isEmpty()) {
            telemetryM.addData("Id", aprilTags.get(0).getFiducialId());
        }
        telemetryM.addData("Auto pipeline", autoPipeline);

//TODO: Dominic's method to aim turret to goal (autoPipeline 0=red goal, 1=blue goal)
        if(!aprilTags.isEmpty() && aprilTags.get(0).getFiducialId() == 20 && autoPipeline == 1){
            telemetry.addLine("ACCESSED");
            if(getDistanceToGoal() > 90) {
                robot.updateLimelightWithXVel(-3.5);
            }else{
                robot.updateLimelightWithXVel(0);
            }
        }else if(!aprilTags.isEmpty() && aprilTags.get(0).getFiducialId() == 24 && autoPipeline == 0){
            if(getDistanceToGoal() > 90) {
                robot.updateLimelightWithXVel(1.5);
            }else{
                robot.updateLimelightWithXVel(0);
            }
        }else{
            robot.turretMotor.setPower(0);
        }

    //manually control the turret rotation; TODO: Dominic to test the turret's speed of rotate
        double turretRotateControl = gamepad2.right_stick_x;
        if (Math.abs(turretRotateControl) > 0.1){
            robot.turretMotor.setPower(0.7 *turretRotateControl);
        }



        robot.slot1RGB.setPosition(ballInSlot(balls[1]));
        robot.slot2RGB.setPosition(ballInSlot(balls[0]));
        robot.slot3RGB.setPosition(ballInSlot(balls[2]));


        //DATA for use with Turret aiming--by PedroPathing coordinate
        //  (remember this is based on starting pose--see line 112 (robot at red endgame box, and both robot and turret point straight forward)
        //IMU yaw=0 is set at initial robot heading when robot is initial turned on.
        //TODO: important: This orientation is carried to all programs later if robot is not turned off.
        //Here, fField angle is set to be

        double robotImuHeadingDeg = robot.imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);
        double robotPedroPathHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        turretTickAt90Degree = 667;       //set limit of Turret position on robot
        turretAngleRelativeToRobot_Deg = robot.turretMotor.getCurrentPosition()*90/turretTickAt90Degree;
        turretAngleRelativeToField_Deg =  robotPedroPathHeadingDeg - turretAngleRelativeToRobot_Deg;
        double goalTurretAngleRelField_Deg = Math.toDegrees(Math.atan((GoalY - follower.getPose().getY())/(GoalX - follower.getPose().getX())));



//TODO: Testing turret Limelight vs. turretAngleRelativeToField_Deg
        //TODO:***** using Limelight to aim turret--use next set of codes and commented out set just below
        //set limit on turret angle relative to robot.
//        if ((turretAngleRelativeToRobot_Deg < -105) || (turretAngleRelativeToRobot_Deg > 105)) {
//            robot.turretMotor.setPower(0);
//            return;
//        } else if (!aprilTags.isEmpty() && aprilTags.get(0).getFiducialId() == 20 && autoPipeline == 1) {
//                telemetryM.addLine("ACCESSED BLUE");
//                turret.update(result);
//                turrentAimingMethod = "Aim turret using LimeLight";
//        } else if (!aprilTags.isEmpty() && aprilTags.get(0).getFiducialId() == 24 && autoPipeline == 0) {
//                telemetryM.addLine("ACCESSED RED");
//                turret.update(result);
//                turrentAimingMethod = "Aim turret using LimeLight";
//        //TODO: need to confirm below, because it can detect the
//        //TODO:***** using odometry to aim turret--use next set of codes and commented out set just above***
//        } else if ((aprilTags.isEmpty())) {
////                || !(aprilTags.get(0).getFiducialId() == 20)
////                || !(aprilTags.get(0).getFiducialId() == 24)){
//            //run this PedroPahting method if turret is within robot oriented angle limits; no detected aprilTags; AND not ID 20 or 24.
//            turret.updateByPedroPath(turretAngleRelativeToField_Deg, goalTurretAngleRelField_Deg);
//            turrentAimingMethod = "Aim turret using PedroPathing";
//        } else {
//            robot.turretMotor.setPower(0);
//            turrentAimingMethod = "No Turret Aim";
//            return;
//        }

        /**
        if ((turretAngleRelativeToRobot_Deg > -120) && (turretAngleRelativeToRobot_Deg < 120)) {
            if (!aprilTags.isEmpty() && aprilTags.get(0).getFiducialId() == 20 && autoPipeline == 1) {
                telemetryM.addLine("ACCESSED BLUE");
                turret.update(result);
                turrentAimingMethod = "Aim turret using LimeLight";
            } else if (!aprilTags.isEmpty() && aprilTags.get(0).getFiducialId() == 24 && autoPipeline == 0) {
                telemetryM.addLine("ACCESSED RED");
                turret.update(result);
                turrentAimingMethod = "Aim turret using LimeLight";
                //TODO: need to confirm below, because it can detect the
                //TODO:***** using odometry to aim turret--use next set of codes and commented out set just above***
            } else if ((aprilTags.isEmpty())
                    || !(aprilTags.get(0).getFiducialId() == 20)
                    || !(aprilTags.get(0).getFiducialId() == 24)){
                //run this PedroPahting method if turret is within robot oriented angle limits; no detected aprilTags; AND not ID 20 or 24.
                turret.updateByPedroPath(turretAngleRelativeToField_Deg, goalTurretAngleRelField_Deg);
                turrentAimingMethod = "Aim turret using PedroPathing";
            } else {
                robot.turretMotor.setPower(0);
                turrentAimingMethod = "No Turret Aim";
                return;
            }
        }
        */







        //close zone 163; far zone: 205
        shooterSpeed = robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES);
//        if (shooterSpeed >= 195 && shooterSpeed <= 215 && !aprilTags.isEmpty()){
//            //detect AprilTag AND shooterSpeed is at HIGH speed target: GREEN, GREEN (both tag and high speed)
////            robot.LEDrightGreen.setState(true);
////            robot.LEDleftGreen.setState(true);
////            robot.LEDrightRed.setState(false);
////            robot.LEDleftRed.setState(false);
//        }
//        else if (shooterSpeed >= 155 && shooterSpeed <= 175 && !aprilTags.isEmpty()) {
//            //detect AprilTag AND shooterSpeed is at LOW speed target: GREEN, RED (both tag and low speed)
//            robot.LEDrightGreen.setState(false);
//            robot.LEDleftGreen.setState(true);
//            robot.LEDrightRed.setState(true);
//            robot.LEDleftRed.setState(false);
//        }
//        else if (!aprilTags.isEmpty()){
//            //detect AprilTag only: ORANGE, ORANGE
//            robot.LEDrightGreen.setState(true);
//            robot.LEDleftGreen.setState(true);
//            robot.LEDrightRed.setState(true);
//            robot.LEDleftRed.setState(true);
//        }
//        else{
//            //NO AprilTag AND not at any speed target: RED, RED  (don't bother shooting.)
//            robot.LEDrightGreen.setState(false);
//            robot.LEDleftGreen.setState(false);
//            robot.LEDrightRed.setState(true);
//            robot.LEDleftRed.setState(true);
//        }
//




        if (robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES) >= 150 && robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES) <= 220 && !aprilTags.isEmpty()){
            robot.statusRGB.setPosition(0.5);
        }
        else {
            robot.statusRGB.setPosition(0);
        }


//        if (gamepad2.a){
//            robot.transferDOWN();
//        }
//        else if (gamepad2.b){
//            robot.transferUP();
//        }

        switch (state) {
            case INTAKE:
                executeOnce = true;
                robot.kickerDOWN();
                if (intakeMode.equals("normal")) {
                    double spindexerServoPosition = robot.getSpindexerServoPosition();
//                    if ((spindexerServoPosition > 0.02 && spindexerServoPosition < 0.07 && robot.spindexerServo.getPosition() == 0) || (spindexerServoPosition > 0.37 && spindexerServoPosition < 0.4 && robot.spindexerServo.getPosition() == 0.37) || (spindexerServoPosition > 0.7 && spindexerServoPosition < 0.75 && robot.spindexerServo.getPosition() == 0.74)) {
//                        intakeSlotPosition = true;
//                    } else {
//                        intakeSlotPosition = false;
//                    }
                    if (spindexerServoPosition > 0.02 && spindexerServoPosition < 0.07 && robot.spindexerServo.getPosition() == 0){
                        currentSlot = 1;
                    }
                    else if (spindexerServoPosition > 0.37 && spindexerServoPosition < 0.4 && robot.spindexerServo.getPosition() == 0.37){
                        currentSlot = 2;
                    }
                    else if (spindexerServoPosition > 0.7 && spindexerServoPosition < 0.75 && robot.spindexerServo.getPosition() == 0.74){
                        currentSlot = 3;
                    }

                    if (currentSlot == 1 && balls[0].equals("NONE")) {
                        robot.spindexerIntakeSlot1();
                    }

                    if (!(ball.equals("NONE"))) {
                        if (currentSlot > 0 && currentSlot <= 3){
                            balls[currentSlot - 1] = ball;
                        }
                        if (currentSlot == 1) {
                            robot.spindexerIntakeSlot2();
                        } else if (currentSlot == 2) {
                            robot.spindexerIntakeSlot3();
                        } else {
                            robot.spindexerShootingSlot1(); // TODO: switch to shooting state? intake out? stop intake?

                        }
                    }
                }
                if (intakeMode.equals("manual")){
                    if (gamepad1.left_trigger >= 0.2){
                        robot.spindexerIntakeSlot1();
                    }
                    else if (gamepad1.left_bumper){
                        robot.spindexerIntakeSlot2();
                    }
                    else if(gamepad1.right_trigger >= 0.2){
                        robot.spindexerIntakeSlot3();
                    }
                }

                if (gamepad1.dpad_up && !(robot.getSpindexerServoPosition() > 0.87)){
                    intakeMode = "normal";
                    robot.intakeIN();
                }
                else if (gamepad1.dpad_right){
                    intakeMode = "manual";
                    robot.intakeIN();
                }
                else if (gamepad1.dpad_down){
                    robot.intakeOUT();
                }

                else if (gamepad1.dpad_left) {
                    robot.intakeSTOP();
                }
                else if (robot.getSpindexerServoPosition() > 0.87){
                    if (gamepad1.dpad_down){
                        robot.intakeOUT();
                    }
                    else{
                        robot.intakeSTOP();
                    }
                }
                else if (gamepad1.a){
                    state = State.SHOOT;
                }

                if (gamepad2.b && (((!aprilTags.isEmpty() && ((aprilTags.get(0).getFiducialId() == 20 && autoPipeline == 1) || (aprilTags.get(0).getFiducialId() == 24 && autoPipeline == 0)))) || !(robot.limelight.isConnected() && robot.limelight.isRunning()))){
                    //robot.auto3Shoot();
                    currentSlot = 4;
                    robot.spindexerShootingSlot1();
                    shootingMode = "all3check";

                }
                //Shoot only next ball
                else if (gamepad2.right_bumper){
                    currentSlot = 4;
                    if (!(balls[1].equals("NONE"))){
                        robot.spindexerShootingSlot1();
                        shootingMode = "checkShootingSlot1";
                    }
                    else if (!(balls[0].equals("NONE"))){
                        robot.spindexerShootingSlot2();
                        shootingMode = "checkShootingSlot2";
                    }
                    else if (!(balls[2].equals("NONE"))){
                        robot.spindexerShootingSlot3();
                        shootingMode = "checkShootingSlot3";
                    }
                }

                //Choose which slot to shoot:
                else if (gamepad2.left_trigger >= 0.2){
                    intakeMode = "none";
                    robot.spindexerShootingSlot1();
                    shootingMode = "checkShootingSlot1";
                }
                else if (gamepad2.left_bumper){
                    intakeMode = "none";
                    robot.spindexerShootingSlot2();
                    shootingMode = "checkShootingSlot2";
                }
                else if (gamepad2.right_trigger >= 0.2){
                    intakeMode = "none";
                    robot.spindexerShootingSlot3();
                    shootingMode = "checkShootingSlot3";
                }
                //Total manual shooting
                else if (gamepad2.y){
                    shootingMode = "manual";
                    state = State.SHOOT;
                }

                if (shootingMode.equals("all3check") && robot.getSpindexerServoPosition() > 0.87){
                    shootingMode = "all3";
                    timer.reset();
                    state = State.SHOOT;
                }
                else if (shootingMode.equals("checkShootingSlot1") && robot.getSpindexerServoPosition() > 0.87 && robot.getSpindexerServoPosition() < 0.91){
                    shootingMode = "onlySlot1";
                    timer.reset();
                    state = State.SHOOT;
                }
                else if (shootingMode.equals("checkShootingSlot2") && robot.getSpindexerServoPosition() > 0.53 && robot.getSpindexerServoPosition() < 0.57){
                    shootingMode = "onlySlot2";
                    timer.reset();
                    state = State.SHOOT;
                }
                else if (shootingMode.equals("checkShootingSlot3") && robot.getSpindexerServoPosition() > 0.2 && robot.getSpindexerServoPosition() < 0.24){
                    shootingMode = "onlySlot3";
                    timer.reset();
                    state = State.SHOOT;
                }

                break;
            case SHOOT:
                //This is for bump correction
                if(executeOnce){
                    yaw = robot.imu.getRobotYawPitchRollAngles().getYaw();
                    executeOnce = false;
//                    bumpCorrectionTimer.reset();
                }

                // Old version of all 3:
                if (shootingMode.equals("all3")){
                    if (timer.milliseconds() > 0 && timer.milliseconds() < 150){
                        robot.spindexerShootingSlot1();
                        robot.kickerUP();
                        //shootingResults.add(robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));
                    }
                    else if(timer.milliseconds() >= 150 && timer.milliseconds() < 350){
                        robot.spindexerShootingSlot1();
                        robot.kickerDOWN();
                    }
                    else if(timer.milliseconds() >= 350 && timer.milliseconds() < 650){
                        robot.spindexerShootingSlot2();
                    }
                    else if(timer.milliseconds() >= 650 && timer.milliseconds() < 850){
                        robot.spindexerShootingSlot2();
                        robot.kickerUP();
                        //shootingResults.add(robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));
                    }
                    else if(timer.milliseconds() >= 850 && timer.milliseconds() < 1000){
                        robot.spindexerShootingSlot2();
                        robot.kickerDOWN();
                    }
                    else if(timer.milliseconds() >= 1000 && timer.milliseconds() < 1300){
                        robot.spindexerShootingSlot3();
                    }
                    else if(timer.milliseconds() >= 1300 && timer.milliseconds() < 1450){
                        robot.spindexerShootingSlot3();
                        robot.kickerUP();
                        //shootingResults.add(robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));
                    }
                    else if(timer.milliseconds() >= 1450 && timer.milliseconds() < 1600){
                        robot.spindexerShootingSlot3();
                        robot.kickerDOWN();
                    }
                    else if(timer.milliseconds() > 1600){
                        robot.spindexerShootingSlot3();
                        Arrays.fill(balls, "NONE");
                        currentSlot = 1;
                        intakeSlotPosition = false;
                        shootingMode = "none";
                    }
                    //robot.shootAll3(timer.milliseconds());

                }
//                else if (shootingMode.equals("all3analog")){
//                    if (timer.milliseconds() > 0 && timer.milliseconds() < 200){
//                        robot.spindexerShootingSlot1();
//                        robot.kickerUP();
//                        //shootingResults.add(robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));
//                    }
//                    else if(timer.milliseconds() >= 200 && timer.milliseconds() < 400){
//                        robot.spindexerShootingSlot1();
//                        robot.kickerDOWN();
//                    }
//                    else if(timer.milliseconds() >= 400 && timer.milliseconds() < 700){
//                        robot.spindexerShootingSlot2();
//                    }
//                    else if(timer.milliseconds() >= 700 && timer.milliseconds() < 900){
//                        robot.spindexerShootingSlot2();
//                        robot.kickerUP();
//                        //shootingResults.add(robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));
//                    }
//                    else if(timer.milliseconds() >= 900 && timer.milliseconds() < 1100){
//                        robot.spindexerShootingSlot2();
//                        robot.kickerDOWN();
//                    }
//                    else if(timer.milliseconds() >= 1100 && timer.milliseconds() < 1400){
//                        robot.spindexerShootingSlot3();
//                    }
//                    else if(timer.milliseconds() >= 1400 && timer.milliseconds() < 1600){
//                        robot.spindexerShootingSlot3();
//                        robot.kickerUP();
//                        //shootingResults.add(robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));
//                    }
//                    else if(timer.milliseconds() >= 1600 && timer.milliseconds() < 1800){
//                        robot.spindexerShootingSlot3();
//                        robot.kickerDOWN();
//                    }
//                    else if(timer.milliseconds() > 1800){
//                        robot.spindexerShootingSlot3();
//                        Arrays.fill(balls, "NONE");
//                        currentSlot = 1;
//                        intakeSlotPosition = false;
//                        shootingMode = "none";
//                    }
//                }
                else if (shootingMode.equals("manual")){
                    if (gamepad2.right_bumper && (((!aprilTags.isEmpty() && ((aprilTags.get(0).getFiducialId() == 20 && autoPipeline == 1) || (aprilTags.get(0).getFiducialId() == 24 && autoPipeline == 0)))) || !(robot.limelight.isRunning() && robot.limelight.isConnected()))){
                        robot.kickerUP();
                        shootingResults.add(robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));
                        timer.reset();
                    }
                    else if(timer.milliseconds() > 150){
                        robot.kickerDOWN();
                    }
                    if (gamepad2.left_trigger >= 0.2){
                        robot.spindexerShootingSlot1();
                    }
                    else if (gamepad2.left_bumper){
                        robot.spindexerShootingSlot2();
                    }
                    else if(gamepad2.right_trigger >= 0.2){
                        robot.spindexerShootingSlot3();
                    }
                }
                else if (shootingMode.equals("onlySlot1")){
                    if (timer.milliseconds() > 0 && timer.milliseconds() < 150){
                        robot.spindexerShootingSlot1();
                        robot.kickerUP();
                    }
                    else if(timer.milliseconds() >= 150 && timer.milliseconds() < 350){
                        robot.spindexerShootingSlot1();
                        robot.kickerDOWN();
                    }
                    else if(timer.milliseconds() > 350){
                        robot.spindexerShootingSlot2();
                        balls[1] = "NONE";
                        currentSlot = 1;
                        intakeSlotPosition = false;
                        shootingMode = "none";
                    }
                }
                else if (shootingMode.equals("onlySlot2")){
                    if (timer.milliseconds() > 0 && timer.milliseconds() < 150){
                        robot.spindexerShootingSlot2();
                        robot.kickerUP();
                    }
                    else if(timer.milliseconds() >= 150 && timer.milliseconds() < 350){
                        robot.spindexerShootingSlot2();
                        robot.kickerDOWN();
                    }
                    else if(timer.milliseconds() > 350){
                        robot.spindexerShootingSlot2();
                        balls[0] = "NONE";
                        currentSlot = 1;
                        intakeSlotPosition = false;
                        shootingMode = "none";
                    }
                }
                else if (shootingMode.equals("onlySlot3")){
                    if (timer.milliseconds() > 0 && timer.milliseconds() < 150){
                        robot.spindexerShootingSlot3();
                        robot.kickerUP();
                    }
                    else if(timer.milliseconds() >= 150 && timer.milliseconds() < 350){
                        robot.spindexerShootingSlot3();
                        robot.kickerDOWN();
                    }
                    else if(timer.milliseconds() > 350){
                        robot.spindexerShootingSlot3();
                        balls[2] = "NONE";
                        currentSlot = 1;
                        intakeSlotPosition = false;
                        shootingMode = "none";
                    }
                }
                if (gamepad1.dpad_up || gamepad2.x){
                    intakeMode = "normal";
                    Arrays.fill(balls, "NONE");
                    currentSlot = 1;
                    intakeSlotPosition = false;
                    shootingMode = "none";
                }
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
            if(getDistanceToGoal() > 90 && result.isValid()) {
                currentSpeed = -212;            //214
            }else if(result.isValid()){
                currentSpeed = -170;
            }

            if(result.isValid()){
                robot.shooterVELO(currentSpeed);
            }
            telemetryM.addData("Target speed", currentSpeed);
            telemetryM.addData("Result", result.isValid());
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
        //telemetryM.update();

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
            telemetryM.addLine("20 SECONDS LEFT");
            endGameRumble20secondsLeftOnce = false;
        }

        //3 rumble for 8 seconds left for hanging
        if ((runtime.seconds() > 110) && endGameRumble10secondsLeftOnce) {
            rumble();
            telemetryM.addLine("10 SECONDS");
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
        telemetryM.addData("Is running and connected", robot.limelight.isRunning() && robot.limelight.isConnected());
//        telemetryM.addData("Hood position", robot.hoodServo.getPosition());
//        telemetryM.addData("Launch angle", HardwareMain.getLaunchAngle());
//        telemetryM.addData("Distance to goal", HardwareMain.getDistanceToGoal());
//        telemetryM.addData("Angle to goal from lightlight", result.getTy());
        telemetryM.addData("Hood position", (double) Math.round(robot.hoodServo.getPosition()) * 10 / 10);
        telemetryM.addData("Launch angle (deg)", (double) Math.round(HardwareMain.getLaunchAngle()) * 10 / 10);
        telemetryM.addData("Distance to goal (inch)", (double) Math.round(HardwareMain.getDistanceToGoal()) * 10 / 10);
        telemetryM.addData("Angle to goal from lightlight (deg)", Math.round(result.getTx()* 100)/100);

        telemetryM.addData("Yaw (deg)",  (double) Math.round(yaw) * 10 / 10);


//TODO: comment out for now to use turret 360 degree PID method instead of previous one from Dominic
//        telemetry.addData("Turret Motor Position:", robot.turretMotor.getCurrentPosition());
//        telemetry.addData("Pinpoint offset 0.1", robot.pinpoint.getVelX(DistanceUnit.MM)*0.1);
//        telemetry.addData("Pinpoint offset 0.01", robot.pinpoint.getVelX(DistanceUnit.MM)*0.01);
//        telemetry.addData("Pinpoint offset 0.001", robot.pinpoint.getVelX(DistanceUnit.MM)*0.001);
//        telemetry.addData("Shooter velocity TPS:", robot.leftShooterMotor.getVelocity());


//        telemetry.addData("Spindexer position", robot.getSpindexerPosition());
//        telemetry.addData("Spindexer position2", robot.spindexerServo.getPosition() == 0.38);
//        String[] target = {"PURPLE BALL", "GREEN BALL", "PURPLE BALL"};
//        telemetry.addData("Current sorted shot", robot.getSortingPosition(target, new String[3], 1));

        telemetryM.addLine("");
        telemetryM.addData("rightShooter AngVel (deg/s?): ", (double) Math.round(robot.rightShooterMotor.getVelocity(AngleUnit.DEGREES) * 10) / 10);
        telemetryM.addData("leftShooter  AngVel (deg/s?): ", (double) Math.round(robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES) * 10) / 10);


        telemetryM.addData("Left digital 0 Purple: ", leftColorPurple);
        telemetryM.addData("Right digital 0 Purple: ", rightColorPurple);
        telemetryM.addData("Left digital 1 Green: ", leftColorGreen);
        telemetryM.addData("Right digital 1 Green: ", rightColorGreen);
        telemetryM.addData("BALL = ", ball);
        telemetryM.addData("Slot 1 = ", balls[0]);
        telemetryM.addData("Slot 2 = ", balls[1]);
        telemetryM.addData("Slot 3 = ", balls[2]);
//        telemetryM.addData("Spindexer Analog Position: ", robot.getSpindexerServoPosition());
        telemetryM.addData("Spindexer Analog Position: ", (double) Math.round(robot.getSpindexerServoPosition() * 100) / 100);

//        Brushland_LeftHue = robot.pin0.getVoltage() / 3.3 * 360;
//        Brushland_RightHue = robot.pin1.getVoltage() / 3.3 * 360;
//        telemetryM.addData("Left hue: ", Brushland_LeftHue);
//        telemetryM.addData("Right hue: ", Brushland_RightHue);


        telemetryM.addLine("");
        telemetryM.debug("loopCycleTime (ms): " + (double) Math.round(loopCycleTime.milliseconds()));

        telemetryM.addLine("");
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive: " + automatedDrive);

        telemetry.addData("Auto pipeline", autoPipeline);

        telemetryM.addLine("");
        telemetryM.addLine("");
        telemetryM.addData("Goal X", GoalX);
        telemetryM.addData("Goal Y", GoalY);
        telemetryM.addData("current Pose X",(double) Math.round(follower.getPose().getX()) * 10 / 10);
        telemetryM.addData("current Pose Y", (double) Math.round(follower.getPose().getY()) * 10 / 10);
        telemetryM.addData("Cur robotImuHeadingDeg = ",  (double) Math.round(robotImuHeadingDeg) * 10 / 10);
        telemetryM.addData("Cur robotPedroPathHeadingDeg = ",  (double) Math.round(robotPedroPathHeadingDeg) * 10 / 10);
        telemetryM.addLine("");
        telemetryM.addLine("");
        double leftFlywheelVelocity = robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES);
        telemetryM.addData("Right Flywheel/Shooter (deg/sec) = ", (double) Math.round(leftFlywheelVelocity) * 10 / 10);
        telemetryM.addData("Cur turretAngleRelativeToField_Deg = ", (double) Math.round(turretAngleRelativeToField_Deg) * 10 / 10);
        telemetryM.addData("Goal turretAngleRelativeToField_Deg = ", (double) Math.round(goalTurretAngleRelField_Deg) * 10 / 10);
        telemetryM.addData("Tx by LimeLight",  Math.round(result.getTx()));
//        telemetryM.addLine(turrentAimingMethod);



        telemetryM.update(telemetry);





//        telemetry.update();

//        telemetry.addLine()
//                .addData("Hue", "%.3f", left_hsvValues[0])
//                .addData("Left BALL = ", ColorBallDetected(left_hsvValues[0]));
//        telemetry.addLine()
//                .addData("Hue", "%.3f", back_hsvValues[0])
//                .addData("Back BALL = ", ColorBallDetected(back_hsvValues[0]));

    }

    @Override
    public void stop(){
        telemetryM.addData("", shootingResults);
        telemetryM.update(telemetry);
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
        if (robot.leftColorPin0.getState() || robot.rightColorPin0.getState() || robot.leftColorPin1.getState() || robot.rightColorPin1.getState()) {
            detectedBall = "BALL";
            return detectedBall;
        }
        else{
            detectedBall = "NONE";
        }
        return detectedBall;
    }
    public static String colorBallDetected(){
        if (robot.leftColorPin0.getState() || robot.rightColorPin0.getState()) {
            detectedBall = "PURPLE BALL";
            return detectedBall;
        }
        else if (robot.leftColorPin1.getState() && robot.rightColorPin1.getState()) {
            detectedBall = "GREEN BALL";
            return detectedBall;
        }
        else{
            detectedBall = "NONE";
        }
        return detectedBall;
    }


    public static double ballInSlot(String ball){
        if (ball.equals("PURPLE BALL")){
            return 0.7;
        }
        else if (ball.equals("GREEN BALL")){
            return 0.5;
        }
        else if (ball.equals("BALL")){
            return 0.63;
        }
        return 0;
    }

    public static boolean toggle(boolean colorToggle){
        colorToggle = !colorToggle;
        return colorToggle;
    }

}