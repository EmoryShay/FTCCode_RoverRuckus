package org.firstinspires.ftc.teamcode.EmoryWasHere.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.LedTest2;

import java.util.List;
import java.util.Locale;

public class RoverRuckusAuton extends OpMode{
    Telemetry.Item patternName;
    public LedTest2.DisplayKind displayKind;
    int step = 0;
    public DcMotor left, right, liftMotor, shoulder, elbow, sweep, dumpLift;
    CRServo latch;
    Servo dump, wrist, marker;
    //sensors
    DigitalChannel magSwitchUp, magSwitchDown, magLatchOpen, magLatchClose;
    AnalogInput pShoulder;
    ColorSensor lSweepC, rSweepC;
    DistanceSensor lSweepD, rSweepD;
    double d, d2;
    boolean lifted = false;
    boolean twoIn = false;
    boolean lessThanL = false; boolean lessThanR = false;
    //gyro define
    BNO055IMU imu;
    //gyro telemetry
    Orientation angles;
    Acceleration gravity;
    String position;

    //used for encoders
    static final double     EXTERNAL_GEARING        = 1.45;
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;  //28  // eg: AndyMark NeverRest40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 26.9 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415))/EXTERNAL_GEARING;

    //TensorFlow
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "Ad2f6Yj/////AAABmXK3skr/8k3JphGJ3t4x40MrNaFtHWPxASQcuhl0pNyZt3t14n1CXxgO7IaCDGEuoLgUCuGvbqurJ8guQ4RPja+5rPoGASD3NZYsHLoePBf2ngj406AkQ/Vnu1kqELtIQ1M/VjForpboQLGVwDPpSWfFgq3JRFQGr5H7sJ3boigJpE6dLbIM+58TVahcgoSqqILbDe5Zq8Epv1YMJz9brXN+AmRvRfE5AkV+N5ohsxud5HCjbEv/pmtPoYXTUiFf7zlQJz+0x9mBbeBaDkwzqoXcrTvJkDyKclwPLUTz9AJiOeMMHI+VfLpflfFtOT/ucVUYV91ogpFnsyXVEpd/C+LW5kwdseqzdflR4EZT9sAw";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    double cvtDegreesL(double degrees) {
        double retVal;
        if (degrees < 0) {
            retVal = 180 + (180 + degrees);
        } else {
            retVal = degrees;
        }
        return retVal;
    }

    double cvtDegreesR(double degrees) {
        double retVal;
        if (degrees <= 0) {
            retVal = Math.abs(degrees);
        } else {
            retVal = 180 + (180-degrees);
        }
        return retVal;
    }

    public void init(){

        //hardware map for config file
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        liftMotor = hardwareMap.dcMotor.get("lift");
        shoulder = hardwareMap.dcMotor.get("arm");
        sweep =hardwareMap.dcMotor.get("sweep");
        //elbow = hardwareMap.dcMotor.get("elbow");
        latch = hardwareMap.crservo.get("latch");
        dump = hardwareMap.servo.get("dump");
        wrist = hardwareMap.servo.get("wrist");
        marker = hardwareMap.servo.get("marker");
        magSwitchUp = hardwareMap.digitalChannel.get("magSwitchUp");
        magSwitchDown = hardwareMap.digitalChannel.get("magSwitchDown");
        magLatchClose = hardwareMap.digitalChannel.get("magLatchClose");
        magLatchOpen = hardwareMap.digitalChannel.get("magLatchOpen");
        pShoulder = hardwareMap.analogInput.get("pShoulder");
        dumpLift = hardwareMap.dcMotor.get("dumpLift");
        lSweepC = hardwareMap.colorSensor.get("lSweep");
        lSweepD = hardwareMap.get(DistanceSensor.class, "lSweep");
        rSweepC = hardwareMap.colorSensor.get("rSweep");
        rSweepD = hardwareMap.get(DistanceSensor.class, "rSweep");

        //set directions
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);
        //elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        latch.setDirection(DcMotorSimple.Direction.REVERSE);
        sweep.setDirection(DcMotorSimple.Direction.FORWARD);
        dumpLift.setDirection(DcMotorSimple.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.REVERSE);

        //brake for wheels
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //gyro telemetry
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        //gyro parameters
        BNO055IMU.Parameters gParameters = new BNO055IMU.Parameters();
        gParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gParameters.loggingEnabled = true;
        gParameters.loggingTag = "IMU";
        //gParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //gyro initialize
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gParameters);

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });//

        //initialize TensorFlow
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        dump.setPosition(0.45);

        //LED lights
        /*displayKind = SupportTensorFlowV1_2.DisplayKind.MANUAL;

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON;
        blinkinLedDriver.setPattern(pattern);

        display = telemetry.addData("Display Kind: ", displayKind.toString());
        patternName = telemetry.addData("Pattern: ", pattern.toString());

        ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);*/
    }

    public void init_loop(){
        //get TensorFlow info
        goldAlign();
        telemetry.addData("order:", position);
    }

    public void loop(){
        d = lSweepD.getDistance(DistanceUnit.CM);
        d2 = rSweepD.getDistance(DistanceUnit.CM);
    }

    /**
     * Stops all motors on the robot
     */
    public void stop(){
        left.setPower(0);
        right.setPower(0);
        liftMotor.setPower(0);
        shoulder.setPower(0);
        latch.setPower(0);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /**
     * The robot will drive straight a given distance
     * @param direction direction of the robot. Forward is moving in the direction of the arm
     * @param distance how far robot goes. "Should" be in inches
     * @param speed how fast the robot goes.
     */
    public void driveStraight(String direction, int distance, double speed) {
        resetEncoder();
        int mDirection = 1;
        if (direction.equals("backward")) {
            mDirection = -1;
        }
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set position
        right.setTargetPosition((int) COUNTS_PER_INCH * distance * mDirection);
        left.setTargetPosition((int) COUNTS_PER_INCH * distance * mDirection);

        //set power
        left.setPower(speed);
        right.setPower(speed);

        //while busy
        while (right.isBusy()&& left.isBusy()) {
            telemetry.addData("left:",left.getCurrentPosition()*COUNTS_PER_INCH);
            telemetry.addData("right:",right.getCurrentPosition()*COUNTS_PER_INCH);
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
        //resetEncoder();
    }

    public void smartSweep() {
        double initTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < initTime + 4800){
            if (d > 9.7 || d2 > 9.7) {
                sweep.setPower(1);
            } else {
                sweep.setPower(0);
            }
        }
    }

    public void autoDumpLift(int time, String direction) {
        double initTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < initTime + time){
            if(direction.equals("up")) {
                dumpLift.setPower(-0.9);
            }else{
                dumpLift.setPower(1);
            }
        }
    }

    public void depositMarker(int time){
        double initTime =  System.currentTimeMillis();
        while(System.currentTimeMillis() < initTime + time){
            marker.setPosition(1);
        }
        marker.setPosition(0);
    }

    /**
     * Resets drive encoders
     */
    public void resetEncoder(){
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * turns off tensorFlow
     */
    public void disableDetector(){
        tfod.shutdown();
    }


    /**
     * sets the position of the mineral using
     * tensorFlow by reading the 2 rightmost minerals
     */
    public void goldAlign()
    {
        //String retVal = "NULL";
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() > 0) {
                    int goldMineralX = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1) {
                        if(goldMineralX>245)
                        {
                            position="RIGHT";
                        }
                        else
                        {
                            position="CENTER";
                        }
                        telemetry.addData("X-Position:",goldMineralX);
                    }
                    else
                    {
                        position="LEFT";
                    }
                    telemetry.addData("Order: ",position);
                }
                telemetry.update();
            }
        }


        // return retVal;
    }




    /**
     *sets the position of the shoulder using the
     * potentiometer
     * @param servoPosition position of the shoulder
     */
    public void mervoShoulder(double servoPosition) {
        double iTime=System.currentTimeMillis();
        double servoPos = servoPosition;
        double voltReading = (float) pShoulder.getVoltage();
        telemetry.addData("voltage:", voltReading);
        double mervoValue = voltReading / 3.25;
        telemetry.addData("servoValue:", mervoValue);


        double inc1 = servoPos - 0.02;
        double inc2 = servoPos + 0.02;

        while ((mervoValue < (servoPos - 0.02) || mervoValue > (servoPos + 0.02))&&System.currentTimeMillis()<iTime+4700) {
            voltReading = (float) pShoulder.getVoltage();
            mervoValue = voltReading / 3.25;
            if (mervoValue < inc1) {
                shoulder.setPower(-0.75);//0.8
                telemetry.addLine("move motor forward");
            } else if (mervoValue > inc2) {
                shoulder.setPower(0.75);
                telemetry.addLine("move motor backward");
            } else {
                shoulder.setPower(0);
            }

        }
        shoulder.setPower(0);

    }


    /**
     * Brings the lift down until it
     * hits the magnetic switch
     */
    public void liftDown() {
        if (!magSwitchDown.getState()) {
            liftMotor.setPower(0);
        }
        else
        {
            liftMotor.setPower(-1);
        }
    }

    /**
     * Brings the lift up until it
     * hits the magnetic switch
     */
    public void liftUp()
    {
        if(!magSwitchUp.getState())
        {
            liftMotor.setPower(0);
        }
        else
        {
            liftMotor.setPower(1);
        }
    }

    /**
     * Opens the latch until it hits the
     * magnetic switch
     */
    public void latchOpen()
    {
        double iTime=System.currentTimeMillis();
        if(!magLatchOpen.getState() || System.currentTimeMillis()>iTime+4800)
        {
            latch.setPower(0);
        }
        else
        {
            latch.setPower(1);
        }
    }

    /**
     * Turns accurate angles depending on the robot's initial position.
     * Negative is counter clockwise
     * Positive is clockwise
     * @param targetAngle the angle you want the robot to
     * @param speed speed of the robot
     */
    public void gyroTest(double targetAngle, double speed) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angles.firstAngle;
        telemetry.addData("angle: ", angles.firstAngle);
        telemetry.update();

        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(currentAngle - targetAngle) > 4) {
            currentAngle = angles.firstAngle;
            telemetry.update();

            if (currentAngle > targetAngle) {//turn left
                telemetry.update();
                left.setPower(-speed);
                right.setPower(speed);
                telemetry.update();
            }
            else if (currentAngle < targetAngle) {
                telemetry.update();
                left.setPower(speed);
                right.setPower(-speed);
                telemetry.update();
            }
            else {
                left.setPower(0);
                right.setPower(0);
                telemetry.update();
            }
            telemetry.update();
            telemetry.addData("angle: ", angles.firstAngle);
        }
        telemetry.update();
        left.setPower(0);
        right.setPower(0);
    }

    public void gyroSmart(double targetAngle){
        double iTime=System.currentTimeMillis();
        double constant = 0.008;//for the sensor bot
        double minSpeed = 0.365;//0.29;//for the sensor bot

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angles.firstAngle;
        telemetry.addData("angle: ", angles.firstAngle);
        telemetry.update();

        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(currentAngle - targetAngle) > 2) {
            currentAngle = angles.firstAngle;
            telemetry.update();

            double speed = 0;

            if(Math.abs(constant*(targetAngle-currentAngle)) > minSpeed) {
                speed = constant*(targetAngle-currentAngle);
            }
            else{
                speed = minSpeed;
            }

            if (currentAngle > targetAngle) {//turn left
                telemetry.update();
                left.setPower(-speed);
                right.setPower(speed);
            }
            else if (currentAngle < targetAngle) {
                telemetry.update();
                left.setPower(speed);
                right.setPower(-speed);
            }
            else {
                left.setPower(0);
                right.setPower(0);
            }
            telemetry.update();
            telemetry.addData("angle: ", angles.firstAngle);
            if(System.currentTimeMillis()>iTime+4500){
                break;
            }
        }
        telemetry.update();
        left.setPower(0);
        right.setPower(0);

    }

    public void gyroBad (double targetAngle, double speed) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = cvtDegreesL(angles.firstAngle);
        telemetry.addData("angle: ", cvtDegreesL(angles.firstAngle));
        telemetry.update();

        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (currentAngle < targetAngle) {
            currentAngle = cvtDegreesL(angles.firstAngle);
            telemetry.update();

                telemetry.update();
                left.setPower(speed);
                right.setPower(-speed);
                telemetry.update();

        }
        telemetry.update();
        left.setPower(0);
        right.setPower(0);
    }

    public void gyro2Left (double targetAngle, double speed) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = cvtDegreesL(angles.firstAngle);
        telemetry.addData("angle: ", cvtDegreesL(angles.firstAngle));
        telemetry.update();

        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(currentAngle - targetAngle) > 2/*(speed * 40)*/) {
            currentAngle = cvtDegreesL(angles.firstAngle);
            telemetry.update();

            if (currentAngle > targetAngle) {//turn left
                telemetry.update();
                left.setPower(-speed);
                right.setPower(speed);
                telemetry.update();
            }
            else if (currentAngle < targetAngle) {
                telemetry.update();
                left.setPower(speed);
                right.setPower(-speed);
                telemetry.update();
            }
            else {
                left.setPower(0);
                right.setPower(0);
                telemetry.update();
            }

        }
        telemetry.update();
        left.setPower(0);
        right.setPower(0);
    }
    public void gyro2Right (double targetAngle, double speed) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = cvtDegreesR(angles.firstAngle);
        telemetry.addData("angle: ", cvtDegreesL(angles.firstAngle));
        telemetry.update();

        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(currentAngle - targetAngle) > 2/*(speed * 40)*/) {
            currentAngle = cvtDegreesR(angles.firstAngle);
            telemetry.update();

            if (currentAngle > targetAngle) {//turn right
                telemetry.update();
                left.setPower(speed);
                right.setPower(-speed);
                telemetry.update();
            }
            else if (currentAngle < targetAngle) {
                telemetry.update();
                left.setPower(-speed);
                right.setPower(speed);
                telemetry.update();
            }
            else {
                left.setPower(0);
                right.setPower(0);
                telemetry.update();
            }

        }
        telemetry.update();
        left.setPower(0);
        right.setPower(0);
    }

    /**
     * a function that is needed to format the gyro angle
     * @param angleUnit
     * @param angle
     * @return
     */
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    /**
     * a function that is needed to format the gyro angle
     * @param degrees
     * @return
     */
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void driveStraightSmart(String driection){

    }

    public void Rotation(float destination) {
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double speed = 0;
        double min = 0.35;
        double max = 0.8;
        double iTime=System.currentTimeMillis();
        //standard current angle
        double heading = cvtDegrees(angles.firstAngle);
        double iHeading=heading;
        /**
         * This will determine the actual intended heading from 0-360
         */

        if (Math.abs(destination)>360) { //check if over 360
            //System.out.println("Destination more than 360 degrees");
            if(destination>0) {//positive
                //System.out.println("positive");
                while(destination>360) {
                    destination-=360;
                    System.out.println(destination);
                }//end while
            }else {//negative
                //  System.out.println("negative");
                while(destination<-360) {
                    destination+=360;
                    System.out.println(destination);
                }//end while
            }//end if else
        }//end greater than 360
        destination = Math.abs(destination);// convert to positive value
        if (destination==0) {//if 360 set to 0 as they are the same heading
            destination = 360;
            //System.out.println("destination 0 converted to 360");
        }
        if (heading==0) {//if 360 set to 0 as they are the same heading
            heading = 360;
            //  System.out.println("heading 0 converted to 360");
        }
        //System.out.println("Start:"+heading+"\nFinal destination:" + destination);

        /**
         * This will determine which direction to turn
         */

        while (heading < destination - 2 || heading > destination + 2) {
            telemetry.addData("heading", heading);
            telemetry.addData("speed: ", speed);
            telemetry.addData("destination:", destination);
            telemetry.update();
            double delta = destination-heading; //the difference between destination and heading;
            heading = cvtDegrees(angles.firstAngle);

            speed = (1 - ((heading) / destination)) * ((destination - heading) * 0.01);


            //if the speed gets under the min speed it will use the min speed
            if (Math.abs(speed) < min && Math.abs(speed) != 0) {
                speed = min;
            }
            //if the speed is over the max it will use max speed
            if(Math.abs(speed) > max){
                speed=max;
            }
            if (!(Math.abs(delta) == 360 || Math.abs(delta) == 0)) {//determine if we are at the intended heading
                if (((delta + 360) % 360) > 180) { //determines if he arc length is longer to the right or left
                    // System.out.println("turn left\n-----");
                    left.setPower(-speed);
                    right.setPower(speed);
                } else {
                    //   System.out.println("turn right\n-----");
                    left.setPower(speed);
                    right.setPower(-speed);
                }
            } else {
                // System.out.println("Already at inteneded heading\n-----");
                left.setPower(0);
                right.setPower(0);
            }
            if(System.currentTimeMillis()>iTime+4500){
                break;
            }
        }
        left.setPower(0);
        right.setPower(0);

    }

    public double cvtDegrees(double heading) {
        /**
         * convert degrees from -180<->180 to 0<->360
         */
        if (heading <0 ) {
            return 360 + heading;
        } else {
            return heading;
        }
    }
 /*
    protected void handleGamepad()
    {
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        /*if (gamepad1.a) {
            setDisplayKind(LEDTest.DisplayKind.MANUAL);
            gamepadRateLimit.reset();
        } else if (gamepad1.b) {
            setDisplayKind(LEDTest.DisplayKind.AUTO);
            gamepadRateLimit.reset();
        } else if ((displayKind == LEDTest.DisplayKind.MANUAL) && (gamepad1.left_bumper)) {
            pattern = pattern.previous();
            displayPattern();
            gamepadRateLimit.reset();
        } else if ((displayKind == LEDTest.DisplayKind.MANUAL) && (gamepad1.right_bumper)) {
            pattern = pattern.next();
            displayPattern();
            gamepadRateLimit.reset();
        }
    }

   protected void setDisplayKind(SupportTensorFlowV1_2.DisplayKind displayKind)
    {
        this.displayKind = displayKind;
        display.setValue(displayKind.toString());
    }

    protected void doAutoDisplay()
    {
        if (ledCycleDeadline.hasExpired()) {
            pattern = pattern.next();
            displayPattern();
            ledCycleDeadline.reset();
        }
    }

    protected void displayPattern()
    {
        blinkinLedDriver.setPattern(pattern);
        patternName.setValue(pattern.toString());
    }*/
 

    /**
     * will stall the program for the inputted amount of time
     * wait time cannot be more than 5 seconds
     * @param milTime the amount of time to wait in seconds
     */
    public void pause(double milTime){
        double iTime =System.currentTimeMillis();
        double fTime=iTime+milTime;
        while(System.currentTimeMillis()<fTime){}
    }

    public void liftPosition(){
        //voltage of shoulder potentiometer
        double sVoltReading = (float) pShoulder.getVoltage();
        //converts voltage of the potentiometer to servo values
        double sServoValue = sVoltReading / 3.25;
        double armStart = 0;
        double iTime=System.currentTimeMillis();

            double sLiftPos =0.255;//.270 //.252
            boolean isLiftHere=false;

            while ((sServoValue < (sLiftPos - .01) || sServoValue > (sLiftPos + .005))&&(System.currentTimeMillis()<iTime+4800)) {
                sVoltReading = (float) pShoulder.getVoltage();
                sServoValue = sVoltReading / 3.25;
                if (sServoValue < (sLiftPos - .01)) {
                    if (isLiftHere) {
                        shoulder.setPower(-0.2);
                    } else {
                        shoulder.setPower(-1);
                    }
                } else if (sServoValue > (sLiftPos + .01)) {
                    if (isLiftHere) {
                        shoulder.setPower(0.2);
                    } else {
                        shoulder.setPower(1);
                    }
                } else {
                    shoulder.setPower(0);
                    wrist.setPosition(1);
                }
                if (sServoValue < 0.58) {
                    wrist.setPosition(1);
                    lifted = true;
                }
                armStart++;
                if ((sServoValue > (sLiftPos - .02)) && (sServoValue < (sLiftPos + .02))) {
                    isLiftHere = true;

                }
            }
    }


}

