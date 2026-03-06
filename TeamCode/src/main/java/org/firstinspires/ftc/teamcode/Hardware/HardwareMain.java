package org.firstinspires.ftc.teamcode.Hardware;

/** copy over from Marcus' branch 1/18/2026
 */

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BLUE_TeleOpV2;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure
//***This was setup for OpMode but can be it used for LinearOpMode also?
//OpMode structure: init(), init_loop(), start(), loop(), stop().  as of SDK 8.1, 1ms between call in loop()
//LinearOpMode structure: runOpMode(), waitForStart(), isStarted(), isStopRequested(), idle(), opModeIsActive(), opModeInInit()

public class HardwareMain {
    public int spindexerPosition = 1;
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
    public GoBildaPinpointDriver pinpoint;

    public DcMotorEx intakeMotor = null;
    public CRServo transferServo = null;
    public Servo kickerServo = null;
    public Servo spindexerServo = null;
    public AnalogInput spindexerServoPosition = null;

    public DcMotorEx rightShooterMotor = null;
    public static DcMotorEx leftShooterMotor = null;
    public DcMotorEx turretMotor = null;
    public Servo hoodServo = null;

    public static Limelight3A limelight = null;

    //public NormalizedColorSensor leftBallColorSensor = null;
    public DigitalChannel leftColorPin0 = null;
    public DigitalChannel leftColorPin1 = null;

    //public NormalizedColorSensor rightBallColorSensor = null;
    public DigitalChannel rightColorPin0 = null;
    public DigitalChannel rightColorPin1 = null;

    //public NormalizedColorSensor backBallColorSensor = null;


    public DigitalChannel LEDrightGreen;
    public DigitalChannel LEDrightRed;
    public DigitalChannel LEDleftRed;
    public DigitalChannel LEDleftGreen;
    public DigitalChannel LEDcenterGreen;
    public DigitalChannel LEDcenterRed;

    double newForward = 0, newRight = 0, driveTheta = 0, r = 0 ;

    final float[] right_hsvValues = new float[3];
    final float[] left_hsvValues = new float[3];
    final float[] back_hsvValues = new float[3];



//declare variables for the "drive" method.
//    double driveTheta, r, newRight, newForward;


    /*Constructor*/
    public HardwareMain() {

    }


    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(50, 0.0001, 0.0005, 12.1);

        //Save reference to Hardware map
        //Sensor.init(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

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
        spindexerServoPosition = hardwareMap.get(AnalogInput.class, "spindexerServoPosition");


        transferServo = hardwareMap.get(CRServo.class, "transferServo");
        transferServo.setDirection(DcMotorSimple.Direction.REVERSE);

        kickerServo = hardwareMap.get(Servo.class,"kickerServo");
        kickerServo.setDirection(Servo.Direction.FORWARD);

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

//        leftBallColorSensor = hardwareMap.get(NormalizedColorSensor.class, "leftBallColorSensor");
//        rightBallColorSensor = hardwareMap.get(NormalizedColorSensor.class, "rightBallColorSensor");
//        backBallColorSensor = hardwareMap.get(NormalizedColorSensor.class, "backBallColorSensor");
//        rightBallColorSensor.setGain(4.5F);
//        leftBallColorSensor.setGain(4.5F);
//        backBallColorSensor.setGain(4.5F);
         leftColorPin0 = hardwareMap.digitalChannel.get("leftColorPin0");
         leftColorPin1 = hardwareMap.digitalChannel.get("leftColorPin1");
         rightColorPin0 = hardwareMap.digitalChannel.get("rightColorPin0");
         rightColorPin1 = hardwareMap.digitalChannel.get("rightColorPin1");

        LEDcenterGreen = hardwareMap.get(DigitalChannel.class, "centerGreen");               //connect to Digital port 4
        LEDcenterRed = hardwareMap.get(DigitalChannel.class, "centerRed");                   //connect to Digital port 5

        LEDleftGreen = hardwareMap.get(DigitalChannel.class, "leftGreen");               //connect to Digital port 2
        LEDleftRed = hardwareMap.get(DigitalChannel.class, "leftRed");                   //connect to Digital port 3

        LEDrightGreen = hardwareMap.get(DigitalChannel.class, "rightGreen");             //connect to Digital port 0
        LEDrightRed = hardwareMap.get(DigitalChannel.class, "rightRed");             //connect to Digital port 1

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
        intakeMotor.setPower(1);
    }


    //public method (function) for spitting out sample
    public void intakeOUT() {
        intakeMotor.setPower(-0.6);
    }

    public void intakeOFF() {
        intakeMotor.setPower(0);

    }

    public void spindexerIntakeSlot1(){
        spindexerPosition = 1;
        spindexerServo.setPosition(0);
    }
    public void spindexerIntakeSlot2(){
        spindexerPosition = 2;
        spindexerServo.setPosition(0.37);
    }
    public void spindexerIntakeSlot3(){
        spindexerPosition = 3;
        spindexerServo.setPosition(0.74);
    }

    public void spindexerShootingSlot1(){
        spindexerPosition = 1;
        spindexerServo.setPosition(0.925);
    }
    public void spindexerShootingSlot2(){
        spindexerPosition = 2;
        spindexerServo.setPosition(0.555);
    }
    public void spindexerShootingSlot3(){
        spindexerPosition = 3;
        spindexerServo.setPosition(0.185);

    }


//    public void setSpindexerPosition(int position){
//        if(position == 1){
//            spindexerServo.setPosition(0);
//            spindexerPosition = 1;
//        }else if(position == 2){
//            spindexerServo.setPosition(0.37);
//            spindexerPosition = 2;
//        }else if(position == 3){
//            spindexerServo.setPosition(0.74);
//            spindexerPosition = 3;
//        }
//    }

    public int getSpindexerPosition(){
        return spindexerPosition;
    }

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

    //For Omniwheel transfer:
    public void transferUP(){
        transferServo.setPower(0.8);

    }
    public void transferDOWN(){
        transferServo.setPower(-0.8);
    }
    public void transferSTOP(){
        transferServo.setPower(0);
    }

    //For kicker servo transfer:
    public void kickerUP(){
        kickerServo.setPosition(0.14);
    }
    public void kickerDOWN(){
        kickerServo.setPosition(0);
    }

    public void hoodOUT(){
        hoodServo.setPosition(0.8);
    } // Goes almost at topmost position

    public void hoodOutFar(){
        try {
            hoodServo.setPosition(getLaunchAngle());
        } catch (Exception e) {
            hoodServo.setPosition(0.55);
        }
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


    public double getSpindexerServoPosition(){
        // get the voltage of our analog line
        // divide by 3.3 (the max voltage) to get a value between 0 and 1
        // multiply by 360 to convert it to 0 to 360
        return spindexerServoPosition.getVoltage() / 3.3;
    }

    public static double getDistanceToGoal(){
        LLResult result = limelight.getLatestResult();
        if(!result.isValid()){
            return BLUE_TeleOpV2.distance;
        }
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
        try {
            double x = getDistanceToGoal() / 39.37;
            double y = 27.25 / 39.37;
            double g = 9.8;

            double v = 7.3;
            if (getDistanceToGoal() < 80
//                    && getDistanceToGoal() > 47
                   )
            {
                v = 6.2;
//            }else if(getDistanceToGoal() < 47){
//                v = 5.8;
            }

            //This is a common value in the equation which I assigned to a variable to cut down on the number of calculations the computer had to do
            double C = (g * x * x) / (2 * v * v);
            double discriminant = Math.sqrt((x * x - 4 * (-C) * (-C - y)));
            double theta1 = Math.atan((-x + discriminant) / (2 * (-C)));
            double theta2 = Math.atan((-x - discriminant) / (2 * (-C)));

            if ((x * x - 4 * (-C) * (-C - y)) < 0) {
                return 0.55;
            }

            double result = -0.0312086 * (Math.min(theta1, theta2) * 180 / Math.PI) + 1.80914;
            if (getDistanceToGoal() > 80) {
                result = result - 0.04;
            }

            if (result < 0) {
                result = 0;
            } else if (result >= 0.76) {
                result = 0.76;
            }
            return result;
        } catch (Exception e) {
               return 0.55;
        }
    }

    public void updateLimelight(double offset){
        LLResult result = limelight.getLatestResult();
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

    public void updateLimelightWithXVel(double offset){
        LLResult result = limelight.getLatestResult();
        if(result.getTx() != 0 && turretMotor.getCurrentPosition() < 833 && turretMotor.getCurrentPosition() > -833) {
            double tx = result.getTx() + offset;
            double min_command = 0.027;
            double Kp = -0.024;
            double heading_error = -tx;
            double steering_adjust = 0.0;
            if (Math.abs(heading_error) > 1.0) {
                if (heading_error < 0) {
                    steering_adjust = (Kp * heading_error + min_command) - (pinpoint.getVelX(DistanceUnit.INCH) * 0.02);
                } else {
                    steering_adjust = Kp * heading_error - min_command - (pinpoint.getVelX(DistanceUnit.INCH) * 0.02);
                }
            }
            turretMotor.setPower(steering_adjust);
        }else{
            turretMotor.setPower(0);
        }
    }

    public int getSortingPosition(String[] target, String[] colors, int shotNumber){
        if(getSpindexerPosition() == 1) {
            if (colors[0].equals(target[shotNumber - 1])) {
                return 1;
            } else if (colors[1].equals(target[shotNumber - 1])) {
                return 2;
            }else if(colors[2].equals(target[shotNumber - 1])){
                return 3;
            }else if(colors[0].equals("NONE")){
                if (!colors[1].equals("NONE")) {
                    return 2;
                }else if(!colors[2].equals("NONE")){
                    return 3;
                }else{
                    return 1;
                }
            }else{
                return 1;
            }
        }else if(getSpindexerPosition() == 2){
            if (colors[0].equals(target[shotNumber - 1])) {
                return 2;
            } else if (colors[1].equals(target[shotNumber - 1])) {
                return 3;
            }else if(colors[2].equals(target[shotNumber - 1])){
                return 1;
            }else if(colors[0].equals("NONE")){
                if (!colors[1].equals("NONE")) {
                    return 3;
                }else if(!colors[2].equals("NONE")){
                    return 1;
                }else{
                    return 2;
                }
            }else{
                return 2;
            }
        }else if(getSpindexerPosition() == 3){
            if (colors[0].equals(target[shotNumber - 1])) {
                return 3;
            } else if (colors[1].equals(target[shotNumber - 1])) {
                return 1;
            }else if(colors[2].equals(target[shotNumber - 1])){
                return 2;
            }else if(colors[0].equals("NONE")){
                if (!colors[2].equals("NONE")) {
                    return 2;
                }else if(!colors[1].equals("NONE")){
                    return 1;
                }else{
                    return 3;
                }
            }else{
                return 3;
            }
        }
        return getSpindexerPosition();
    }


}



