package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;

import java.util.List;

@Configurable
@TeleOp(name= "RED_TeleOpV2 V1.0", group = "RED")

public class RED_TeleOpV2 extends BLUE_TeleOpV2 {

    //int autoPipeline;    // TODO: set pipeline to Red


    @Override
    public void init() {
        autoPipeline = 0;  // TODO: set pipeline to Red
        GoalX = 129;
        GoalY = 128;
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
