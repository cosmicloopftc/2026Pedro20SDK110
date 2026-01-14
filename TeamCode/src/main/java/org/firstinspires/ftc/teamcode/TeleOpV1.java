package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareDrivetrain;
import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


@Configurable
@TeleOp
public class TeleOpV1 extends OpMode {


    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.2;
    ElapsedTime timer = new ElapsedTime();

    public static HardwareMain robot = new HardwareMain();
    HardwareDrivetrain robotDrivetrain = new HardwareDrivetrain();
    enum State{
        INTAKE,
        SHOOT
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

    }

    public void init_loop() {
        if(gamepad2.left_bumper){
            robot.limelight.pipelineSwitch(0);
            telemetry.addLine("Red pipeline initialized");
        }else if(gamepad2.right_bumper){
            robot.limelight.pipelineSwitch(1);
            telemetry.addLine("Blue pipeline initialized");
        }
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        follower.update();
        robot.limelight.start();
    }
    @Override
    public void loop() {
        //AprilTagTracking
        LLResult result = robot.limelight.getLatestResult();
        //TODO get the right numbers for the position of the shooter
        if(result.getTx() != 0 && robot.turretMotor.getCurrentPosition() < 300 && robot.turretMotor.getCurrentPosition() > -300) {
            double tx = result.getTx();
            //This is to adjust for the  fact that aiming at the april tag from a long distance is not actually
            //where we want to aim
            if(getDistanceToGoal() > 80){
                //TODO get a good value for this
                tx+=0.5;
            }
            double min_command = 0.001;
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
        }else{
            //TODO test to see if this is useful and works
            if(result.getTx() == 0 && robot.turretMotor.getTargetPosition() != 0){
                robot.turretMotor.setTargetPosition(0);
            } else if(robot.turretMotor.getCurrentPosition() <= -300 && gamepad1.left_stick_x > 0){
                robot.turretMotor.setPower(gamepad1.left_stick_x * 0.2);
            }else if(robot.turretMotor.getCurrentPosition() >= 300 && gamepad1.left_stick_x < 0){
                robot.turretMotor.setPower(gamepad1.left_stick_x * 0.2);
            }else {
                robot.turretMotor.setPower(0);
            }
        }

//        if (gamepad2.a){
//            robot.transferDOWN();
//        }
//        else if (gamepad2.b){
//            robot.transferUP();
//        }

        switch (state) {
            case INTAKE:
                robot.transferDOWN();

                if (gamepad1.dpad_up){
                    robot.intakeIN();
                    robot.transferIN();
                }
                else if (gamepad1.dpad_down){
                    robot.intakeOUT();
                    robot.transferOUT();
                }
                else if (gamepad1.dpad_left || gamepad1.dpad_right) {
                    robot.intakeSTOP();
                    //robot.transferOFF();
                }
                else if (gamepad1.a){
                    state = State.SHOOT;
                }

                break;
            case SHOOT:
                robot.transferIN();

                if (gamepad1.a){
                    robot.transferUP();
                    timer.reset();
                }
                else if (timer.milliseconds() > 800)
                {
                    robot.transferDOWN();
                }
                if (gamepad1.left_trigger > 0.2 || gamepad1.left_bumper){
                    robot.intakeIN();
                }
                else{
                    robot.intakeSTOP();
                }
                if (gamepad1.dpad_up || gamepad1.dpad_down){
                    state = State.INTAKE;
                }
                break;
        }

        if (gamepad2.y){
            robot.shooterON();
        }
        else if (gamepad2.a){
            robot.shooterOFF();
        }
        if(gamepad2.dpad_down){
            robot.hoodIN();
        }else if(gamepad2.dpad_left){
            robot.hoodOutFar();
        }
        else if(gamepad2.dpad_right){
            robot.hoodOutClose();
        }
        else if(gamepad2.dpad_up){
            robot.hoodServo.setPosition(getLaunchAngle());
        }
        else if (gamepad1.ps){
            robot.transferOFF();
        }


        //Call this once per loop
        follower.update();
        telemetryM.update();
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    true // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    gamepad1.left_stick_y * slowModeMultiplier,
                    gamepad1.left_stick_x * slowModeMultiplier,
                    gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }
//        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }
//        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }
//        //Slow Mode
//        if (gamepad1.rightBumperWasPressed()) {
//            slowMode = !slowMode;
//        }
//        //Optional way to change slow mode strength
//        if (gamepad1.xWasPressed()) {
//            slowModeMultiplier += 0.25;
//        }
//        //Optional way to change slow mode strength
//        if (gamepad2.yWasPressed()) {
//            slowModeMultiplier -= 0.25;
//        }
        telemetry.addData("Hood position", robot.hoodServo.getPosition());
        telemetry.addData("Launch angle", getLaunchAngle());
        telemetry.addData("Distance to goal", getDistanceToGoal());
        telemetry.addData("Angle to goal from lightlight", result.getTy());
        telemetry.addData("Turret Motor Position:", robot.turretMotor.getCurrentPosition());
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
    public static double getDistanceToGoal(){
        LLResult result = robot.limelight.getLatestResult();
        double targetOffsetAngle_Vertical = result.getTy();
        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 30.45;  //32.39;

        // distance from the center of the Limelight lens to the floor
        //TODO get this variable
        double limelightLensHeightInches = 11.25;

        // distance from the target to the floor
        double goalHeightInches = 38.75;

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

        return (90-(Math.min(theta1, theta2) * 180/Math.PI))/360 - 0.01; //37 = 0
    }
}