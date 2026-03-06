package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
// @TeleOp(name= "Template V1.0", group = "RED")

public class OpMode_Template extends BLUE_TeleOpV2 {


    int autoPipeline;


    @Override
    public void init() {
        super.init();
        autoPipeline = 0;  // TODO: set pipeline to Red
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
