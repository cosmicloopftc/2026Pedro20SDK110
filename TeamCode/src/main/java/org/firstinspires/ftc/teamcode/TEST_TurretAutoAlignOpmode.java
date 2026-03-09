package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.HardwareDrivetrainNOTusingPedroPath;
import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;

@Configurable
@TeleOp(name= "TEST_TurretAutoAlign V1.1", group = "Test")

public class TEST_TurretAutoAlignOpmode extends OpMode {

    public static HardwareMain robot = new HardwareMain();
    HardwareDrivetrainNOTusingPedroPath robotDrivetrain = new HardwareDrivetrainNOTusingPedroPath();
    private TurretMechanism turret = new TurretMechanism();

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime endRuntime = new ElapsedTime();
    private ElapsedTime loopCycleTime = new ElapsedTime();


    double[] PstepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    double[] DstepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int PstepIndex = 2;
    int DstepIndex = 2;

    int maxAllowTurretTick = 829, minAllowTurretTick = - -840, turretTickAt90Degree = 667;       //set limit of Turret position on robot
    double turretAngleRelativeToRobot_Deg, turretAngleRelativeToField_Deg;

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    @IgnoreConfigurable
    static PoseHistory poseHistory;
    public static double distance;
    int autoPipeline =0;                    // TODO: set pipeline to red
    String turrentAimingMethod;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot.init(hardwareMap);
        //robot.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotDrivetrain.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        poseHistory = follower.getPoseHistory();
        //Set up bulk data reading
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        follower.setPose(new Pose(0, 0, 0));





        robot.limelight.pipelineSwitch(0);
        turret.init(hardwareMap);

        telemetryM.addLine("Initialized all mechanisms");
        telemetryM.update(telemetry);;

    }

    @Override
    public void init_loop() {

        telemetryM.addLine("Initialized all mechanisms");
        LLResult id24 = robot.limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> aprilTags = id24.getFiducialResults();
        telemetryM.addData("Is empty", aprilTags.isEmpty());
        if(!aprilTags.isEmpty()) {
            telemetryM.addData("Id", aprilTags.get(0).getFiducialId());
        }
        telemetryM.update(telemetry);;

    }


    public void start() {
        turret.resetTimer();
        robot.limelight.start();
        resetRuntime();


        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        follower.update();



    }

    @Override
    public void loop() {
        loopCycleTime.reset();
        telemetryM.addData("RunTime: ", (double) Math.round(getRuntime()* 10) / 10);

        LLResult id24 = robot.limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> aprilTags = id24.getFiducialResults();






//TODO: testing turret aiming by LimeLight vs. Pedropathing location
        //turret.update(id24);
        //DATA for use with Turret aiming--by PedroPathing coordinate
        //  (remember this is based on starting pose--see line 112 (robot at red endgame box, and both robot and turret point straight forward)
        //IMU yaw=0 is set at initial robot heading when robot is initial turned on.
        //    This orientation is carried to all programs later if robot is not turned off.
        //Here, fField angle is set to be
        double robotImuHeadingDeg = robot.imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);
        double robotPedroPathHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        turretTickAt90Degree = 667;       //set limit of Turret position on robot
        turretAngleRelativeToRobot_Deg = robot.turretMotor.getCurrentPosition()*90/turretTickAt90Degree;
        turretAngleRelativeToField_Deg =  robotPedroPathHeadingDeg - turretAngleRelativeToRobot_Deg;
        double GoalX = 129;                    //RED goal coordinates
        double GoalY = 128;
        double goalTurretAngleRelField_Deg = Math.toDegrees(Math.atan((GoalY - follower.getPose().getY())/(GoalX - follower.getPose().getX())));

        if ((turretAngleRelativeToRobot_Deg > -120) && (turretAngleRelativeToRobot_Deg < 120)) {
            if (!aprilTags.isEmpty() && aprilTags.get(0).getFiducialId() == 20 && autoPipeline == 1) {
                telemetryM.addLine("ACCESSED BLUE");
                turret.update(id24, 0);
                turrentAimingMethod = "Aim turret using LimeLight";
            } else if (!aprilTags.isEmpty() && aprilTags.get(0).getFiducialId() == 24 && autoPipeline == 0) {
                telemetryM.addLine("ACCESSED RED");
                turret.update(id24, 0);
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






        if (gamepad1.yWasPressed()) {
            PstepIndex = (PstepIndex + 1) % PstepSizes.length;
        }
        if (gamepad1.aWasPressed()) {
            DstepIndex = (DstepIndex + 1) % DstepSizes.length;
        }


        if (gamepad1.dpadLeftWasPressed()) {
            turret.setkP(turret.getkP() - PstepSizes[PstepIndex]);
        }
        if (gamepad1.dpadRightWasPressed()) {
            turret.setkP(turret.getkP() + PstepSizes[PstepIndex]);
        }

        if (gamepad1.dpadUpWasPressed()) {
            turret.setkD(turret.getkD() + DstepSizes[DstepIndex]);
        }
        if (gamepad1.dpadDownWasPressed()) {
            turret.setkD(turret.getkD() - DstepSizes[DstepIndex]);
        }


        if (id24 != null) {
            telemetryM.addData("cur ID", robot.limelight);

        } else {
            telemetryM.addLine("No Tag Detected. Stopping Turret Motor");
        }

        telemetryM.addLine("---------------------------------");
        //telemetryM.addLine(turret.getPIDmethod());
        telemetry.addData("Tuning P", "%.5f (D-Pad L/R)", turret.getkP());
        telemetry.addData("Tuning D", "%.5f (D-Pad U/D)", turret.getkD());
        telemetry.addData("Step Size for P", "%.5f (Y Button)", PstepSizes[PstepIndex]);
        telemetry.addData("Step Size for D", "%.5f (a Button)", DstepSizes[DstepIndex]);
        telemetry.addLine("");
        telemetry.addData("motor power", "%.2f", turret.getPower());
        telemetry.addData("Tx--for tuning", "%.5f", id24.getTx());
        telemetry.addData("error", "%.5f", (-id24.getTx()));
        telemetry.addData("Distance to goal", (double) Math.round(HardwareMain.getDistanceToGoal()* 10) / 10);
        telemetry.addLine("");
        telemetry.addData("RunTime (s): ", (double) Math.round(getRuntime()* 10) / 10);
        telemetry.addData("loopCycleTime (ms): ", (double) Math.round(loopCycleTime.milliseconds() * 100) / 100);

//        telemetryM.addData("Tuning P  (D-Pad L/R): " , "%.5f turret.getkP()");
//        telemetryM.debug("Tuning D  (D-Pad U/D): ", "%.5f turret.getkD()");
//        telemetryM.debug("Step Size  (Y Button): ", "%.5f stepSizes[stepIndex]");

        telemetryM.addLine("---------------------------------");
        telemetryM.addData("Distance to goal", HardwareMain.getDistanceToGoal());
        telemetryM.addData("Ty, Angle to goal from lightlight", (double) Math.round(id24.getTy()* 100) / 100);
        telemetryM.addData("Tx--for tuning", (double) Math.round(id24.getTx()* 100) / 100);
        telemetryM.addData("Is empty", aprilTags.isEmpty());
        if(!aprilTags.isEmpty()) {
            telemetryM.addData("Id", aprilTags.get(0).getFiducialId());
        }
        telemetryM.addLine("");
        telemetryM.addData("RunTime (s): ", (double) Math.round(getRuntime()* 10) / 10);
        telemetryM.debug("loopCycleTime (ms): " + (double) Math.round(loopCycleTime.milliseconds() * 100) / 100);


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
        telemetryM.addData("Tx by LimeLight",  (double) Math.round(id24.getTx() * 100 / 100));
        telemetryM.addLine(turrentAimingMethod);
        telemetryM.update(telemetry);

    }
}
