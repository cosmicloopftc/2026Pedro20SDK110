package org.firstinspires.ftc.teamcode;
//TODO:  need to add changes Dominic made on 1/22/2026 Wed-Thurs (master_endJan18LATEST_DCP branch)
//             in HardwareMain, and in TeleOpV2.java


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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.HardwareDrivetrainNOTusingPedroPath;
import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;

/**1/18/2026  correct manual move control (used PedroPathing teleOp method)
 * identify code to slow down manual move (slowModeMultiplier, slowMode) ; auto path (automatedDrive)
 *
 */

@Configurable
@TeleOp(name = "TeleOpV2 v1")
public class TeleOpV2 extends OpMode {
    int autoPipeline;
    public static double distance = 0;
    public static boolean shooterOn = false;
    public static int currentSpeed = -210;
    public static boolean test = false;

    boolean endGameRumble20secondsLeftOnce = true;
    boolean endGameRumble10secondsLeftOnce = true;
    Gamepad.RumbleEffect customRumbleEffect;

    private ElapsedTime runtime = new ElapsedTime();

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
    }

    public void init_loop() {
//        telemetry.addLine("G2.left_bumper to initialize RED Limelight");
//        telemetry.addLine("");
//        telemetry.addLine("G2.right_bumper to initialize RED Limelight");
//        telemetry.addLine("");

        if(gamepad2.left_bumper){
            robot.limelight.pipelineSwitch(0);
            telemetry.addLine("Red pipeline initialized");
        }else if(gamepad2.right_bumper){
            robot.limelight.pipelineSwitch(1);
            telemetry.addLine("Blue pipeline initialized");
        }
        telemetry.addData("AutoPipeline", autoPipeline);
        telemetry.update();
        autoPipeline = robot.limelight.getLatestResult().getPipelineIndex();

    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        follower.update();
        robot.limelight.pipelineSwitch(3);
        robot.limelight.start();
    }
    @Override
    public void loop() {
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
            if(getDistanceToGoal() > 80) {
                robot.updateLimelight(-3);
            }else{
                robot.updateLimelight(0);
            }
        }else if(!aprilTags.isEmpty() && aprilTags.get(0).getFiducialId() == 24 && autoPipeline == 0){
            if(getDistanceToGoal() > 80) {
                robot.updateLimelight(3);
            }else{
                robot.updateLimelight(0);
            }
        }else{
            robot.turretMotor.setPower(0);
        }

//        if (gamepad2.a){
//            robot.transferDOWN();
//        }
//        else if (gamepad2.b){
//            robot.transferUP();
//        }

        switch (state) {
            case INTAKE:
                robot.spindexerPosition1();
                robot.transferDOWN();
                if (gamepad2.b && (!aprilTags.isEmpty() && (aprilTags.get(0).getFiducialId() == 20 && autoPipeline == 1) || (aprilTags.get(0).getFiducialId() == 24 && autoPipeline == 0))){
                    //robot.auto3Shoot();
                    shootingMode = "all3";
                    timer.reset();
                    state = State.SHOOT;
                }
                else if (gamepad2.right_bumper){
                    shootingMode = "manual";
                    state = State.SHOOT;
                }

                if (gamepad1.dpad_up || gamepad1.left_trigger > 0.2){
                    robot.intakeIN();
                    robot.intakeServoIN();
                }
                else if (gamepad1.dpad_down){
                    robot.intakeOUT();
                    robot.intakeServoOUT();
                }
                if (gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.left_bumper) {
                    robot.intakeSTOP();
                    robot.intakeServoSTOP();
                }
                else if (gamepad1.a){
                    state = State.SHOOT;
                }

                break;
            case SHOOT:
                //robot.transferIN();
                if (shootingMode.equals("all3")){
                    robot.intakeServoIN();
                    if (timer.milliseconds() > 0 && timer.milliseconds() < 250){
                        robot.spindexerPosition1();
                        robot.transferUP();                    }
                    else if(timer.milliseconds() >= 250 && timer.milliseconds() < 500){
                        robot.spindexerPosition1();
                        robot.transferDOWN();
                    }
                    else if(timer.milliseconds() >= 500 && timer.milliseconds() < 900){
                        robot.spindexerPosition2();
                    }
                    else if(timer.milliseconds() >= 900 && timer.milliseconds() < 1150){
                        robot.spindexerPosition2();
                        robot.transferUP();                    }
                    else if(timer.milliseconds() >= 1150 && timer.milliseconds() < 1400){
                        robot.spindexerPosition2();
                        robot.transferDOWN();
                    }
                    else if(timer.milliseconds() >= 1400 && timer.milliseconds() < 2400){
                        robot.spindexerPosition3();
                    }
                    else if(timer.milliseconds() >= 2400 && timer.milliseconds() < 2650){
                        robot.spindexerPosition3();
                        robot.transferUP();                    }
                    else if(timer.milliseconds() >= 2650 && timer.milliseconds() < 2900){
                        robot.spindexerPosition3();
                        robot.transferDOWN();
                    }
                    else if(timer.milliseconds() > 2900){
                        robot.spindexerPosition1();
                        shootingMode = "none";
                    }
                }
                else if (shootingMode.equals("manual")){
                    robot.intakeServoIN();
                    if (gamepad2.right_bumper && (!aprilTags.isEmpty() && (aprilTags.get(0).getFiducialId() == 20 && autoPipeline == 1) || (aprilTags.get(0).getFiducialId() == 24 && autoPipeline == 0))){
                        robot.transferUP();
                        timer.reset();
                    }
                    else if(timer.milliseconds() > 200){
                        robot.transferDOWN();
                    }
                    if (gamepad2.left_trigger >= 0.2){
                        robot.spindexerPosition1();
                    }
                    else if (gamepad2.left_bumper){
                        robot.spindexerPosition2();
                    }
                    else if(gamepad2.right_trigger >= 0.2){
                        robot.spindexerPosition3();
                    }
                    if (gamepad1.dpad_up || gamepad2.x){
                        shootingMode = "none";
                    }
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
//            robot.shooterON(); // Far launch zone, bounce out
            //robot.shooterVELO(-190); // Far launch zone, bounce out
            //robot.shooterVELO(-185); // Far launch zone, bounce out
            shooterOn = true;
        }
        else if (gamepad2.dpad_down){
            robot.hoodMID();
            //robot.shooterVELO(-167); // Near launch zone, equivalent to 0.54 power
//            robot.shooterVELO(-160); // Near launch zone, equivalent to 0.52 power
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
            robot.shooterVELO(currentSpeed);
            shooterOn = true;
        }
        if(shooterOn){
            if(getDistanceToGoal() > 80 && result.isValid()) {
                currentSpeed = -207;
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
        if(currentSpeed == -165){
            telemetry.addLine("SOMETHING IS WRONG");
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

        telemetry.addData("rightShooter AngVel (deg/s?): ", robot.rightShooterMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("leftShooter  AngVel (deg/s?): ", robot.leftShooterMotor.getVelocity(AngleUnit.DEGREES));

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

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

}