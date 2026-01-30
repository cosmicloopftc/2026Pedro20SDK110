package org.firstinspires.ftc.teamcode;

//TODO: need correction--the velocity and datawriting were incorrectly placed here instead of the shooter.

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Datalogger;
import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;

import java.util.List;

//FTC Thunderbolts (S0acramento, CA) mentor program structure
//1:01:slider example adjusts the motor speed for the slide for it to reach a set Encoder value.
//OpMode structure: init(), init_loop(), start(), loop(), stop().  as of SDK 8.1, 1ms between call in loop()
//LinearOpMode structure: runOpMode(), waitForStart(), isStarted(), isStopRequested(), idle(), opModeIsActive(), opModeInInit()


@TeleOp (name= "Test_MotorEncoder", group = "Test")

public class MotorEncoderTest extends OpMode {
    //This method will be called once, when the INIT button is pressed.
    public static HardwareMain robot = new HardwareMain();

    ElapsedTime runtime = new ElapsedTime(); ;
    ElapsedTime lastButtonPushTimeA;
    ElapsedTime lastButtonPushTimeB;
    ElapsedTime timeLastPush = new ElapsedTime(); ;

    private boolean isPushed_dPad_up, isPushed_dPad_down;
    int  currentEncoder;
    int desiredVelocity = 140;              // degree/second
    int desiredRPM;

    Datalog datalog;
    VoltageSensor battery;
    public int i = 0;



    public void init() {
        robot.init(hardwareMap);
        //Set up bulk data reading
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        battery = hardwareMap.voltageSensor.get("Control Hub");
        datalog = new Datalog("datalog_01");
        datalog.opModeStatus.set("INIT");
        datalog.battery.set(battery.getVoltage());
        datalog.writeLine();
        telemetry.setMsTransmissionInterval(50);
    }

    public void init_loop () {
        desiredRPM = (int) Math.round(desiredVelocity*60/360);

        telemetry.addLine("Move gamepad1.left_stick_y UP or DOWN");
        telemetry.addLine("to set Velocity. Press START to begin.");
        telemetry.addData("desired Velocity = ", desiredVelocity);
        telemetry.addData("desired rpm = ", desiredRPM);
        telemetry.update();

        desiredVelocity = desiredVelocity + (int) Math.round((gamepad1.left_stick_y)*10);

        }

    public void start () {

            runtime.reset();
            timeLastPush =runtime;

            datalog.opModeStatus.set("RUNNING");

        }


    public void loop () {
//        isPushed_dPad_up = gamepad1.dpadUpWasReleased();
//        isPushed_dPad_down = gamepad1.dpadDownWasPressed();
//
//
////        currentButtonStateB = gamepad1.dpad_down;
////
//        if (isPushed_dPad_up ) {
//            currentEncoder = currentEncoder + 10;
//            timeLastPush = runtime;
//            isPushed_dPad_up = false;
//            robot.turretMotor.setTargetPosition(currentEncoder);
//        }
//        if (isPushed_dPad_down) {
//            currentEncoder = currentEncoder - 10;
//            timeLastPush = runtime;
//            isPushed_dPad_down = false;
//            robot.turretMotor.setTargetPosition(currentEncoder);
//        }

        datalog.loopCounter.set(i);
        datalog.battery.set(battery.getVoltage());
        datalog.data_runtime.set(runtime.milliseconds());
        datalog.data_currentVelocity.set(robot.turretMotor.getVelocity(AngleUnit.DEGREES));


        robot.turretMotor.setVelocity(desiredVelocity,AngleUnit.DEGREES);
        telemetry.addData("runtime (milliseconds) = ", runtime.milliseconds());
        telemetry.addData("Motor current Velocity (deg/sec) = ", robot.turretMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.update();


    }


    public void stop () {
    }




    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("Loop Counter");
        public Datalogger.GenericField data_runtime = new Datalogger.GenericField("runtime");
        public Datalogger.GenericField data_desiredVelocity = new Datalogger.GenericField("desiredVelocity");
        public Datalogger.GenericField data_desiredRPM  = new Datalogger.GenericField("desiredRPM");
        public Datalogger.GenericField data_currentVelocity = new Datalogger.GenericField("currentVelocity");
        public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");


        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            opModeStatus,
                            loopCounter,
                            data_runtime,
                            data_desiredVelocity,
                            data_desiredRPM,
                            data_currentVelocity,
                            battery
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }


}
