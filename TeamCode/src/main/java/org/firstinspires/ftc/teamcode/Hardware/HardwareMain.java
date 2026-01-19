package org.firstinspires.ftc.teamcode.Hardware;

/** copy over from Marcus' branch 1/18/2026
 */

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure
//***This was setup for OpMode but can be it used for LinearOpMode also?
//OpMode structure: init(), init_loop(), start(), loop(), stop().  as of SDK 8.1, 1ms between call in loop()
//LinearOpMode structure: runOpMode(), waitForStart(), isStarted(), isStopRequested(), idle(), opModeIsActive(), opModeInInit()

public class HardwareMain {
//**ADD assignment of variables here for subsequent connected device.

/* example use of enum from FTC Thunderbolts (Sacramento, CA) mentor's program structure
    enum hwMapState {
        DRIVETRAIN_MOTOR_ONLY,
        MOTOR_OUTAKE_SLIDER,
        MOTOR_OUTAKE_SLIDER_ONLY
    }
*/

    HardwareMap hwMap = null;

    //*Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    //*Important to set initial value to Motor and Servo objects, even if null.

    //voltage sensor
    public VoltageSensor batteryVoltageSensor;
    public double batteryVoltageSensorThreshold = 9.0;

    //*Setup IMU sensor object.
    public IMU imu;

    public DcMotorEx intakeMotor = null;

    public CRServo intakeLeftTransfer = null;
    public CRServo intakeRightTransfer = null;
    public Servo spindexerServo = null;
    public Servo transferServo = null;

    public DcMotorEx rightShooterMotor = null;
    public DcMotorEx leftShooterMotor = null;
    public DcMotorEx turretMotor = null;
    public Servo hoodServo = null;

    public Limelight3A limelight = null;

    double newForward = 0, newRight = 0, driveTheta = 0, r = 0 ;




//declare variables for the "drive" method.
//    double driveTheta, r, newRight, newForward;


    /*Constructor*/
    public HardwareMain() {

    }


    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        //Save reference to Hardware map
        //Sensor.init(hardwareMap);


        //TODO: Define and initialize IMU sensor on new Control Hub--new orientation of the control hub
        imu = hardwareMap.get(IMU.class, "imu");
        //Define how the hub is mounted on robot to get correct Yaw, Pitch and Roll values.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        //Initialize IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //    imu.resetYaw();

        //map and setup mode of Intake Motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //intake motor behaviors
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setPower(0);

        //map and setup mode of Intake Servos
        intakeLeftTransfer = hardwareMap.get(CRServo.class, "intakeLeftTransfer");
        intakeLeftTransfer.setDirection(DcMotorSimple.Direction.FORWARD); //Change if wrong
        intakeRightTransfer = hardwareMap.get(CRServo.class, "intakeRightTransfer");
        intakeRightTransfer.setDirection(DcMotorSimple.Direction.REVERSE); //Change if wrong

        spindexerServo = hardwareMap.get(Servo.class,"spindexerServo");
        spindexerServo.setDirection(Servo.Direction.REVERSE);

        transferServo = hardwareMap.get(Servo.class,"transferServo");

        //map and setup mode of Shooter motor
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        rightShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);        //use this for run at .setvelocity
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);               //use this for run at.setvelocity
        //rightShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);     //use this for run at .setpower
        rightShooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightShooterMotor.setPower(0);

        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        leftShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);     //use this for run at .setvelocity
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);            //use this for run at .setvelocity
        //leftShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);  //use this for run at .setpower
        leftShooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftShooterMotor.setPower(0);


        //map and setup mode of Turret motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setPower(0);

        //map nad setup mode of Hood servo
        hoodServo = hardwareMap.get(Servo.class,"hoodServo");

        //limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");



//?unknown source        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        //batteryVoltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 2");       //GeorgeFIRST kickoff video

    }


    public void start(){
        //PUT functions here

    }

    public void loop(){

    }

    public void stop () {

    }

    //reading Control/Expansion Hub Data
    public Number[] bulkRead() {
//        int newLeftEncoder = -LBack_Motor.getCurrentPosition();       //0. direction is reverse
//        int newRightEncoder = RBack_Motor.getCurrentPosition();       //1.
//        int newCenterEncoder = -RFront_Motor.getCurrentPosition();    //2. direction is reverse
//        int sliderMotorRightPosition = Slider.Slider_Motor_Right.getCurrentPosition();    //3
//        int sliderMotorLeftPosition = Slider.Slider_Motor_Left.getCurrentPosition();      //4
//        double Slider_ServoRightPosition = Slider.Slider_Servo_Arm.getPosition();       //5
//        double Slider_ServoLeftPosition = Slider.Slider_Servo_Dropper.getPosition();         //6
//        double Intake_ServoPosition = Intake.Intake_Servo.getPosition();                  //7
//        int Intake_MotorPosition = Intake.Intake_Motor.getCurrentPosition();              //8
//        double Airplane_ServoPosition = Airplane.Airplane_Servo.getPosition();            //9
//        int Suspension_MotorPosition = Suspension.Suspension_Motor.getCurrentPosition();  //10
//        double Suspension_ServoPosition = Suspension.Suspension_Servo.getPosition();      //11
//
//        double batteryVoltage = Double.POSITIVE_INFINITY;
//        for (VoltageSensor sensor : hwMap.voltageSensor) {
//            double voltage = sensor.getVoltage();
//            if (voltage > 0) {
//                batteryVoltage= Math.min(batteryVoltage, voltage);
//            }
//        }
//
        Number[] bulkArray = new Number[20];
//        bulkArray[0] = newLeftEncoder;
//        bulkArray[1] = newRightEncoder;
//        bulkArray[2] = newCenterEncoder;
//        bulkArray[3] = sliderMotorRightPosition;
//        bulkArray[4] = sliderMotorLeftPosition;
//        bulkArray[5] = Slider_ServoRightPosition;
//        bulkArray[6] = slider_ServoDropperPosition;
//        bulkArray[7] = Intake_ServoPosition;
//        bulkArray[8] = Intake_MotorPosition;
//        bulkArray[9] = Airplane_ServoPosition;
//        bulkArray[10]= Suspension_MotorPosition;
//        bulkArray[11]= Suspension_ServoPosition;

        return bulkArray;
    }

    //public method (function) for intaking in sample
    public void intakeIN() {
        intakeMotor.setPower(0.6);
    }


    //public method (function) for spitting out sample
    public void intakeOUT() {
        intakeMotor.setPower(-0.6);

    }

    public void spindexerPosition1(){
        spindexerServo.setPosition(0);
    }
    public void spindexerPosition2(){
        spindexerServo.setPosition(0.375);
    }
    public void spindexerPosition3(){
        spindexerServo.setPosition(0.75);
    }
    public void transferIN(){
        intakeLeftTransfer.setPower(0.8);
        intakeRightTransfer.setPower(0.8);
    }
    public void transferOUT(){
        intakeLeftTransfer.setPower(-0.8);
        intakeRightTransfer.setPower(-0.8);
    }

    public void transferOFF(){
        intakeLeftTransfer.setPower(0);
        intakeRightTransfer.setPower(0);
    }
    //public method (function) for stopping the intake
    public void intakeSTOP() {
        intakeMotor.setPower(0);
    }

    //Algorithm to calculate speed? - this is temporary!!!!
    public void shooterON() {
        //rightShooterMotor.setPower(-0.2);
        //leftShooterMotor.setPower(0.2);    // 1/18/2026: Confirmed, motors spin in opposite to each others.
        //TODO:  need to find optimal .setVelocity below
        rightShooterMotor.setVelocity(-30,AngleUnit.DEGREES);       //value degrees per seconds
        leftShooterMotor.setVelocity(30,AngleUnit.DEGREES);         //value degrees per seconds
    }
    public void shooterOFF() {
        rightShooterMotor.setPower(0);
        leftShooterMotor.setPower(0);
    }

    public void transferUP(){
        transferServo.setPosition(0.042);

    }
    public void transferDOWN(){
        transferServo.setPosition(0.008);
    }

    public void hoodOutFar(){
        hoodServo.setPosition(0.17);
    }
    public void hoodOutClose(){
        hoodServo.setPosition(0.18);
    }

    public void hoodIN(){
        hoodServo.setPosition(0);
    }

    public void moveHoodUp(){
        if(!(hoodServo.getPosition() >= 0.25)) {
            hoodServo.setPosition(hoodServo.getPosition() + 0.01);
        }
    }
    public void moveHoodDown(){
        if(!(hoodServo.getPosition() <= 0.05)) {
            hoodServo.setPosition(hoodServo.getPosition() - 0.01);
        }
    }

    public void auto3Shoot(){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        if (timer.milliseconds() > 0 && timer.milliseconds() < 800){
            spindexerPosition1();
            //move kicker up?
        }
        else if(timer.milliseconds() >= 400 && timer.milliseconds() < 800){
            spindexerPosition1();
            //move kicker down?
        }
        else if(timer.milliseconds() >= 800 && timer.milliseconds() < 1200){
            spindexerPosition2();
        }
        else if(timer.milliseconds() >= 1200 && timer.milliseconds() < 1600){
            spindexerPosition2();
            //move kicker up?
        }
        else if(timer.milliseconds() >= 1600 && timer.milliseconds() < 2000){
            spindexerPosition2();
            //move kicker down?
        }
        else if(timer.milliseconds() >= 2000 && timer.milliseconds() < 2400){
            spindexerPosition3();
        }
        else if(timer.milliseconds() >= 2400 && timer.milliseconds() < 2800){
            spindexerPosition3();
            //move kicker up?
        }
        else if(timer.milliseconds() >= 2800 && timer.milliseconds() < 3200){
            spindexerPosition3();
            //move kicker down?
        }
//        else{
//            spindexerPosition1();
//        }
    }
}