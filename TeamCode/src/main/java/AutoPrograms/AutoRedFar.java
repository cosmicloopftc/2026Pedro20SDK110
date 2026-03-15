package AutoPrograms; // make sure this aligns with class location

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Far(extend AutoBlueFar) v1.3", group = "Red")

public class AutoRedFar extends AutoBlueFar {

    public AutoRedFar() {
        this.imuOffsetAtStart = 90;            //translate IMU to Pedropath: positive yaw direction is counter-clockwise

        this.autoLimelightPipeline = 0;
        this.startPose = new Pose(88, 8, Math.toRadians(90));           // Start Pose of our robot.
        this.pickup1Pose = new Pose(92.5, 35, Math.toRadians(0));       // Scoring Pose of our robot.
        this.pickup1Pose2 = new Pose(134, 35, Math.toRadians(0));       //old from DCM: Pose(132.5, 35, Math.toRadians(0)
        this.scorePose = new Pose(88, 13, Math.toRadians(60));
        this.parkPose = new Pose(97, 19, Math.toRadians(90));

        this.pickup2Pose = new Pose(131, 25, Math.toRadians(310));
        this.pickup2Pose2 = new Pose(135, 11, Math.toRadians(270));
   }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        super.init();
        robot.limelight.pipelineSwitch(0);                   //pipeline to BLUE = 1 (RED = 0)
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
        offset = -1;         //TODO: Ask Dominic to determine
        super.loop();
    }
    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        super.stop();
    }
}