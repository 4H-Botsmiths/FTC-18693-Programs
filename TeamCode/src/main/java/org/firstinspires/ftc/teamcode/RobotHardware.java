package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public DcMotorEx LeftDrive = null;
    public DcMotorEx RightDrive = null;
    public DcMotorEx LeftShooter = null;
    public DcMotorEx RightShooter = null;

    public Servo Servo1 = null;
    public Servo Servo2 = null;
    public Servo Servo3 = null;
    public Servo Servo4 = null;
    public Servo Servo5 = null;
    public Servo Servo6 = null;

    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RobotHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LeftDrive = hwMap.get(DcMotorEx.class, "Motor_0");
        RightDrive = hwMap.get(DcMotorEx.class, "Motor_1");
        LeftShooter = hwMap.get(DcMotorEx.class, "Motor_2");
        RightShooter = hwMap.get(DcMotorEx.class, "Motor_3");

        // Define and initialize ALL installed servos.
        Servo1 = hwMap.get(Servo.class, "Motor_0");
        Servo2 = hwMap.get(Servo.class, "Motor_1");
        Servo3 = hwMap.get(Servo.class, "Motor_2");
        Servo4 = hwMap.get(Servo.class, "Motor_3");
        Servo5 = hwMap.get(Servo.class, "Motor_4");
        Servo6 = hwMap.get(Servo.class, "Motor_5");

        // Set all motors to zero power
        LeftDrive.setPower(0);
        RightDrive.setPower(0);
        LeftShooter.setPower(0);
        RightShooter.setPower(0);

        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);

        LeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        RightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LeftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


    }
}}