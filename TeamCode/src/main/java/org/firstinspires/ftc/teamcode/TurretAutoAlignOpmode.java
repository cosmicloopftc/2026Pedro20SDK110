package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name= "TurretAutoAlignTest", group = "Test")

public class TurretAutoAlignOpmode extends OpMode {

    public static HardwareMain robot = new HardwareMain();
    private TurretMechanism turret = new TurretMechanism();

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;



    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;





    @Override
    public void init() {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot.init(hardwareMap);
        robot.limelight.pipelineSwitch(0);
        turret.init(hardwareMap);

        telemetryM.addLine("Initialized all mechanisms");


    }

    public void start() {
        turret.resetTimer();
        robot.limelight.start();
    }

    @Override
    public void loop() {
        LLResult id24 = robot.limelight.getLatestResult();

        turret.update(id24);


        if (gamepad1.yWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            turret.setkP(turret.getkP() - stepSizes[stepIndex]);
        }

        if (gamepad1.dpadRightWasPressed()) {
            turret.setkP(turret.getkP() + stepSizes[stepIndex]);
        }

        if (gamepad1.dpadUpWasPressed()) {
            turret.setkP(turret.getkD() + stepSizes[stepIndex]);
        }

        if (gamepad1.dpadDownWasPressed()) {
            turret.setkP(turret.getkD() - stepSizes[stepIndex]);
        }


        if (id24 != null) {
            telemetryM.addData("cur ID", robot.limelight);

        } else {
            telemetryM.addLine("No Tag Detected. Stopping Turret Motor");
        }

        telemetryM.addLine("---------------------------------");
        telemetryM.debug("Tuning P", "%.5f (D-Pad L/R)", turret.getkP());
        telemetryM.debug("Tuning D", "%.5f (D-Pad U/D)", turret.getkD());
        telemetryM.debug("Step Size", "%.5f (Y Button)", stepSizes[stepIndex]);

//        telemetry.addData("Tuning P", "%.5f (D-Pad L/R)", turret.getkP());
//        telemetry.addData("Tuning D", "%.5f (D-Pad U/D)", turret.getkD());
//        telemetry.addData("Step Size", "%.5f (Y Button)", stepSizes[stepIndex]);
    }
}
