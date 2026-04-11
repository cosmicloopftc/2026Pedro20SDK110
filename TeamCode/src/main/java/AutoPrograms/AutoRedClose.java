package AutoPrograms; // make sure this aligns with class location

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Close(extend AutoBlueFar) v1.3", group = "Red")

public class AutoRedClose extends AutoBlueClose {

    public AutoRedClose() {
        this.imuOffsetAtStart = -144;            //translate IMU to Pedropath: positive yaw direction is counter-clockwise

        this.startPose = new Pose(118, 130.5, Math.toRadians(-144)); // Start Pose of our robot.
        this.scorePose = new Pose(86, 102, Math.toRadians(30));
        this.pickup1Pose = new Pose(96, 84, Math.toRadians(0)); // Scoring Pose of our robot.
        this.pickup1Pose2 = new Pose(124, 84, Math.toRadians(0));

        this.pickup2Pose = new Pose(96, 57, Math.toRadians(0));
        this.pickup2Pose2 = new Pose(133, 57, Math.toRadians(0));

        this.parkPose = new Pose(119,84, Math.toRadians(90));

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