package org.firstinspires.ftc.teamcode.Hardware;

/** copy over from Marcus' branch 1/18/2026
 */

import static org.firstinspires.ftc.teamcode.TeleOpV2.getLaunchAngle;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    public CRServo intakeServo = null;
//    public CRServo intakeLeftTransfer = null;   OLD
//    public CRServo intakeRightTransfer = null;  OLD
    public Servo spindexerServo = null;
    public Servo transferServo = null;
    public AnalogInput transferServoPosition = null;

    public DcMotorEx rightShooterMotor = null;
    public static DcMotorEx leftShooterMotor = null;
    public DcMotorEx turretMotor = null;
    public Servo hoodServo = null;

    public static Limelight3A limelight = null;

    double newForward = 0, newRight = 0, driveTheta = 0, r = 0 ;



//declare variables for the "drive" method.
//    double driveTheta, r, newRight, newForward;


    /*Constructor*/
    public HardwareMain() {

    }


    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(105, 0.003, 0.005, 13.9);

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
//        intakeLeftTransfer = hardwareMap.get(CRServo.class, "intakeLeftTransfer");           OLD
//        intakeLeftTransfer.setDirection(DcMotorSimple.Direction.FORWARD); //Change if wrong  OLD
//        intakeRightTransfer = hardwareMap.get(CRServo.class, "intakeRightTransfer");         OLD
//        intakeRightTransfer.setDirection(DcMotorSimple.Direction.REVERSE); //Change if wrong OLD

        spindexerServo = hardwareMap.get(Servo.class,"spindexerServo");
        spindexerServo.setDirection(Servo.Direction.REVERSE);


        transferServo = hardwareMap.get(Servo.class, "transferServo");
//        transferServoPosition = hardwareMap.get(AnalogInput.class, "transferServoPosition");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        //map and setup mode of Shooter motor
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        rightShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);        //use this for run at .setvelocity
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);               //use this for run at.setvelocity
        //rightShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);     //use this for run at .setpower
        rightShooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightShooterMotor.setPower(0);

        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        leftShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);     //use this for run at .setvelocity
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);            //use this for run at .setvelocity
        //leftShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);  //use this for run at .setpower
        leftShooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftShooterMotor.setPower(0);


        //map and setup mode of Turret motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setPower(0);

        //map and setup mode of Hood servo
        hoodServo = hardwareMap.get(Servo.class,"hoodServo");


        //map IMU
        imu = hardwareMap.get(IMU.class, "imu");


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

    public void intakeOFF() {
        intakeMotor.setPower(0);

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

    public int getSpindexerPosition(){
        if(spindexerServo.getPosition() == 0) {
            return 1;
        } else if(spindexerServo.getPosition() == 0.375){
            return 2;
        }else if(spindexerServo.getPosition() == 0.75){
            return 3;
        } else{
            return 4;
        }
    }
//    public void transferIN(){
//        intakeLeftTransfer.setPower(0.8);
//        intakeRightTransfer.setPower(0.8);
//    }
//    public void transferOUT(){
//        intakeLeftTransfer.setPower(-0.8);
//        intakeRightTransfer.setPower(-0.8);
//    }

//    public void transferOFF(){
//        intakeLeftTransfer.setPower(0);
//        intakeRightTransfer.setPower(0);
//    }
    //public method (function) for stopping the intake
    public void intakeSTOP() {
        intakeMotor.setPower(0);
    }

    //Algorithm to calculate speed? - this is temporary!!
    // !!
    public void shooterON() {
        rightShooterMotor.setPower(-0.65);
        leftShooterMotor.setPower(0.65);    // 1/18/2026: Confirmed, motors spin in opposite to each others.
        //TODO:  need to find optimal .setVelocity below
//        rightShooterMotor.setVelocity(-30,AngleUnit.DEGREES);       //value degrees per seconds
//        leftShooterMotor.setVelocity(30,AngleUnit.DEGREES);         //value degrees per seconds
    }
    public void shooterVELO(double RateDegPerSec){
        rightShooterMotor.setVelocity(RateDegPerSec, AngleUnit.DEGREES);       //value degrees per seconds
        leftShooterMotor.setVelocity(-RateDegPerSec, AngleUnit.DEGREES);         //value degrees per seconds
    }
    public void shooterOFF() {
        rightShooterMotor.setPower(0);
        leftShooterMotor.setPower(0);
    }

    public void transferUP(){
        transferServo.setPosition(0);

    }
    public void transferDOWN(){
        transferServo.setPosition(0.24);
    }

    public void hoodOUT(){
        hoodServo.setPosition(0.8);
    } // Goes almost at topmost position

    public void hoodOutFar(){
        hoodServo.setPosition(getLaunchAngle());
    }

    public void hoodMID(){
        hoodServo.setPosition(0.43);
    } //Middle position

    public void hoodIN(){
        hoodServo.setPosition(0);
    } //At bottommost position

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

    public void intakeServoIN(){
        intakeServo.setPower(0.7);
    }
    public void intakeServoOUT(){
        intakeServo.setPower(-0.7);
    }
    public void intakeServoSTOP(){
        intakeServo.setPower(0);
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

    public static double getDistanceToGoal(){
        LLResult result = limelight.getLatestResult();
        double targetOffsetAngle_Vertical = result.getTy();
        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 19.48;  //32.39;

        // distance from the center of the Limelight lens to the floor
        //TODO get this variable
        double limelightLensHeightInches = 11.5;

        // distance from the target to the floor
        double goalHeightInches = 29.25;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    public static double getLaunchAngle(){
        double x = getDistanceToGoal()/39.37;
        double y = 27.25/39.37;
        double g = 9.8;

        double v = 7.3;
        if (getDistanceToGoal() < 80) {
            v = 6.2;
        }

        //This is a common value in the equation which I assigned to a variable to cut down on the number of calculations the computer had to do
        double C = (g*x*x) / (2*v*v);
        double discriminant = Math.sqrt((x*x - 4*(-C)*(-C-y)));
        double theta1 = Math.atan((-x + discriminant) / (2*(-C)));
        double theta2 = Math.atan((-x - discriminant) / (2*(-C)));

        if((x*x - 4*(-C)*(-C-y)) < 0){
            return 0.55;
        }

        double result = -0.0312086 * (Math.min(theta1, theta2) * 180 / Math.PI) + 1.80914;
        if (getDistanceToGoal() > 80) {
            result = result - 0.03;
        }

        if(result < 0){
            result = 0;
        }else if(result >= 0.86){
            result = 0.84;
        }
        return result;
    }

    public void updateLimelight(double offset){
        LLResult result = limelight.getLatestResult();
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(result.getTx() != 0 && turretMotor.getCurrentPosition() < 300 && turretMotor.getCurrentPosition() > -300) {
            double tx;
            tx = result.getTx() + offset;

            double min_command = 0.027;
            double Kp = -0.024;
            double heading_error = -tx;
            double steering_adjust = 0.0;
            if (Math.abs(heading_error) > 1.0) {
                if (heading_error < 0) {
                    steering_adjust = Kp * heading_error + min_command;
                } else {
                    steering_adjust = Kp * heading_error - min_command;
                }
            }
            turretMotor.setPower(steering_adjust);
        }else{
            turretMotor.setPower(0);
        }
    }

}