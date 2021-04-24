package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public CRServo Servo1 = null;
    public CRServo Servo2 = null;
    public CRServo Servo3 = null;
    public CRServo Servo4 = null;
    public CRServo Servo5 = null;
    public CRServo Servo6 = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public RobotHardware() {

    }

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
        Servo1 = hwMap.get(CRServo.class, "Servo_0");
        Servo2 = hwMap.get(CRServo.class, "Servo_1");
        Servo3 = hwMap.get(CRServo.class, "Servo_2");
        Servo4 = hwMap.get(CRServo.class, "Servo_3");
        Servo5 = hwMap.get(CRServo.class, "Servo_4");
        Servo6 = hwMap.get(CRServo.class, "Servo_5");

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftShooter.setPower(0);
        rightShooter.setPower(0);

        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);

        leftDrive.setDirection(DcMotorEx.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotorEx.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

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
        leftShooter.setZeroPowerBehavior(float);
        rightShooter.setZeroPowerBehavior(float);


    }
}