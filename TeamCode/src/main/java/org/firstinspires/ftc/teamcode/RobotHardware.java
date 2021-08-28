package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import static android.os.SystemClock.sleep;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class RobotHardware {
    /* Public OpMode members. */
    public DcMotorEx leftDrive = null;
    public DcMotorEx rightDrive = null;
    public DcMotorEx leftShooter = null;
    public DcMotorEx rightShooter = null;

    public CRServo rampBottom = null;
    public CRServo rampMiddle = null;
    public CRServo rampTop = null;
    public CRServo clawArm = null;
    public CRServo clawHand = null;

    public TouchSensor touchBottom = null;
    public TouchSensor touchTop = null;
    public LED redLight = null;
    public LED blueLight = null;
    public LED greenLight = null;
    public LED spareLight1 = null;
    public LED spareLight2 = null;
    public LED spareLight3 = null;
    public LED spareLight4 = null;
    public LED spareLight5 = null;


    public ColorSensor color1 = null;
    public Rev2mDistanceSensor distanceLeft = null;
    public Rev2mDistanceSensor distanceRight = null;

    public BNO055IMU gyro = null;
    public BNO055IMU.Parameters parameters = null;
    public VoltageSensor voltageSensor = null;


    public double shootTPR = 28;
    public double driveTPR = 288;
    public double shootRPS = 16;
    public double driveRPS = 2;
    public final double maxServoPower = 1;
    public final double maxShootVelocity = shootTPR*shootRPS;
    public final double maxDriveVelocity = driveTPR*driveRPS;
    public double servoPower = maxServoPower;
    public double shootVelocity = maxShootVelocity;
    public double driveVelocity = maxDriveVelocity;
    public double lowBattery = 10.5;
    public double reallyLowBattery = 9.5;
    public double circumferenceMM = 280;
    public double circumferenceIN = 11;
    public final double driveTickPerInch = driveTPR/circumferenceIN;
    public final double driveMilimeterPerTick = driveTPR/circumferenceMM;
    public final double armRatio = 6;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    //public RobotHardware() {

    //}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotorEx.class, "Motor_0");
        rightDrive = hwMap.get(DcMotorEx.class, "Motor_1");
        leftShooter = hwMap.get(DcMotorEx.class, "Motor_2");
        rightShooter = hwMap.get(DcMotorEx.class, "Motor_3");

        // Define and initialize ALL installed servos.
        rampBottom = hwMap.get(CRServo.class, "Servo_0");
        rampMiddle = hwMap.get(CRServo.class, "Servo_1");
        rampTop = hwMap.get(CRServo.class, "Servo_2");
        clawArm = hwMap.get(CRServo.class, "Servo_3");
        clawHand = hwMap.get(CRServo.class, "Servo_4");

        // Define and initialize ALL installed touch sensors.
        touchBottom = hwMap.get(TouchSensor.class, "Touch_0");
        touchTop = hwMap.get(TouchSensor.class, "Touch_1");

        // Define and initialize ALL installed lights.
        redLight = hwMap.get(LED.class, "Light_0");
        blueLight = hwMap.get(LED.class, "Light_1");
        greenLight = hwMap.get(LED.class, "Light_2");
        spareLight1 = hwMap.get(LED.class, "Light_3");
        spareLight2 = hwMap.get(LED.class, "Light_4");
        spareLight3 = hwMap.get(LED.class, "Light_5");
        spareLight4 = hwMap.get(LED.class, "Light_6");
        spareLight5 = hwMap.get(LED.class, "Light_7");
        // Define and initialize ALL installed distance/light sensors.
        color1 = hwMap.get(ColorSensor.class, "Color_0");
        distanceLeft = hwMap.get(Rev2mDistanceSensor.class, "Distance_1");
        distanceRight = hwMap.get(Rev2mDistanceSensor.class, "Distance_2");

        // Define and initialize ALL internal sensors.
        gyro = hwMap.get(BNO055IMU.class, "imu");
        voltageSensor = hwMap.get(VoltageSensor.class, "Control Hub");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.temperatureUnit      = BNO055IMU.TempUnit.FARENHEIT;
        gyro.initialize(parameters);



        // make sure the imu gyro is calibrated before continuing.


        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftShooter.setPower(0);
        rightShooter.setPower(0);


        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //PIDF Caliration
        double Shooter_Motors_F = 32767 / maxShootVelocity, Shooter_Motors_P = 0.1 * Shooter_Motors_F, Shooter_Motors_I = 0.1 * Shooter_Motors_P;
        leftShooter.setVelocityPIDFCoefficients(Shooter_Motors_P, Shooter_Motors_I, 0, Shooter_Motors_F);
        rightShooter.setVelocityPIDFCoefficients(Shooter_Motors_P, Shooter_Motors_I, 0, Shooter_Motors_F);

        double Drive_Motors_F = 32767 / maxDriveVelocity, Drive_Motors_P = 0.1 * Drive_Motors_F, Drive_Motors_I = 0.1 * Drive_Motors_P;
        leftDrive.setVelocityPIDFCoefficients(Drive_Motors_P, Drive_Motors_I, 0, Drive_Motors_F);
        rightDrive.setVelocityPIDFCoefficients(Drive_Motors_P, Drive_Motors_I, 0, Drive_Motors_F);

        while (!gyro.isGyroCalibrated())
        {
            sleep(50);
        }
    }
}