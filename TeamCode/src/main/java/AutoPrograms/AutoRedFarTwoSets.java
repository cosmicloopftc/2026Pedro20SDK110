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

import java.util.Arrays;
import java.util.List;

@Configurable
@Autonomous(name = "RedFar Two Sets V1.1", group = "Examples")
public class AutoRedFarTwoSets extends AutoBlueFarTwoSets {
    double imuOffsetAtStart = -135;            //turn right 135 degree to match PedroPath: positive yaw direction is counter-clockwise

    public static double offset = 1.5;

//    public Pose startPose; // Start Pose of our robot.
//    public Pose pickup1Pose; // Scoring Pose of our robot.
//    public Pose pickup1Pose2;
//    public Pose scorePose;
//    public Pose parkPose;
//
//    public Pose pickup2Pose;
//    public Pose pickup2Pose2;

    public int autoLimelightPipeline = 0;          //pipeline to BLUE = 1 (RED = 0)

    public AutoRedFarTwoSets() {
        this.autoLimelightPipeline = 0;
        this.startPose = new Pose(88, 8, Math.toRadians(90));           // Start Pose of our robot.
        this.pickup1Pose = new Pose(92.5, 35, Math.toRadians(0));       // Scoring Pose of our robot.
        this.pickup1Pose2 = new Pose(134, 35, Math.toRadians(0));       //old from DCM: Pose(132.5, 35, Math.toRadians(0)
        this.scorePose = new Pose(88, 13, Math.toRadians(60));
        this.parkPose = new Pose(97, 19, Math.toRadians(90));

        this.pickup2Pose = new Pose(131, 25, Math.toRadians(310));
        this.pickup2Pose2 = new Pose(135, 11, Math.toRadians(270));
    }


    private PathChain goToPickup1, goToPickup2, grabPickup1, scorePickup1, grabPickup2, scorePickup2, scorePreload, park;




    public void buildPaths() {
        super.buildPaths();
    }

    public void autonomousPathUpdate() {
       super.autonomousPathUpdate();
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        super.setPathState(pState);
    }





    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        super.init();
        robot.limelight.pipelineSwitch(0);
        offset = 1.5;
    }


    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
       super.init_loop();
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        super.start();
    }




    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        super.loop();
    }





    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        super.stop();
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


//    public static void drawOnlyCurrent() {
//        try {
//            Drawing.drawRobot(follower.getPose());
//            Drawing.sendPacket();
//        } catch (Exception e) {
//            throw new RuntimeException("Drawing failed " + e);
//        }
//    }

//    public static void draw() {
//        Drawing.drawDebug(follower);
//    }



}