package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;

import java.util.List;

@Configurable
@TeleOp(name= "RED_TeleOpV2 V1.3", group = "A")

public class RED_TeleOpV2 extends BLUE_TeleOpV2 {

    @Override
    public void init() {
        //TODO: need to adjust this starting point--for now it is at BLUE endgame box--robot and turret point straight forward.
        //TODO: set startingPose to start TeleOp--tx from Auto?
        //blue Endgame box = (105, 33, ...); //red Endgame box = (38, 33, ....)
        //TODO: need to confirm with Dominic:  BLUE goal ?? (14,128);   RED goal (129, 128);
        //For now, at start of TeleOp, set RED FAR robot to be at BLUE Endgame box = (38. 33, ....) and point up to goal Y direction
        //For now, at start of TeleOp, set RED NEAR robot to be at ??? and point up to goal Y direction

        autoPipeline = 0;  // TODO: set autoPipeline to Red = 0 (BLUE = 1)
        GoalX = 129;                    //RED goal coordinates
        GoalY = 128;
        startingPose = new Pose(105,33, Math.toRadians(90)); //    Consider having the robot at this BLUE endgame box area.
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }


    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    public void stop(){
        super.stop();
    }
}