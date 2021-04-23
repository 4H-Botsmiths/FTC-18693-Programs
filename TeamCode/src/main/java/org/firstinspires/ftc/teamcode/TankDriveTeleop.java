package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TankDriveTeleop", group = "ForRobot")

public class TankDriveTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor motor1 = hardwareMap.get(DcMotor.class, "Motor_0");
        DcMotor motor2 = hardwareMap.get(DcMotor.class, "Motor_1");
        DcMotorEx motor3 = hardwareMap.get(DcMotorEx.class, "Motor_2");
        DcMotorEx motor4 = hardwareMap.get(DcMotorEx.class, "Motor_3");
        CRServo servo1 = hardwareMap.get(CRServo.class, "Servo_0");
        CRServo servo2 = hardwareMap.get(CRServo.class, "Servo_1");
        CRServo servo3 = hardwareMap.get(CRServo.class, "Servo_2");
        CRServo servo4 = hardwareMap.get(CRServo.class, "Servo_3");
        CRServo servo5 = hardwareMap.get(CRServo.class, "Servo_4");
        //Motor1.setTargetPosition(0);
        //Motor2.setTargetPosition(0);
        //Motor3.setTargetPosition(0);
        //Motor4.setTargetPosition(0);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3.setDirection(DcMotorEx.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double launcher;
        double pickup = 0;
        double middle = 0;
        double feeder = 0;
        double claw;
        double drawbridge;
        double auto = 0;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double LeftY = Math.pow(-gamepad1.left_stick_y, 3);
                double RightY = Math.pow(-gamepad1.right_stick_y, 3);
                if (gamepad1.b) {
                    if (pickup == 0) {
                        if (gamepad1.right_bumper) {
                            servo1.setDirection(DcMotorSimple.Direction.REVERSE);
                        } else {
                            servo1.setDirection(DcMotorSimple.Direction.FORWARD);
                        }
                        servo1.setPower(1);
                        pickup = 1;
                    } else {
                        servo1.setPower(0);
                        pickup = 0;
                    }
                    sleep(500);
                }
                if (gamepad1.y) {
                    if (middle == 0) {
                        if (gamepad1.right_bumper) {
                            servo2.setDirection(DcMotorSimple.Direction.REVERSE);
                        } else {
                            servo2.setDirection(DcMotorSimple.Direction.FORWARD);
                        }
                        servo2.setPower(1);
                        middle = 1;
                    } else {
                        servo2.setPower(0);
                        middle = 0;
                    }
                    sleep(500);
                }
                if (gamepad1.x) {
                    if (feeder == 0) {
                        if (gamepad1.right_bumper) {
                            servo3.setDirection(DcMotorSimple.Direction.REVERSE);
                        } else {
                            servo3.setDirection(DcMotorSimple.Direction.FORWARD);
                        }
                        servo3.setPower(1);
                        feeder = 1;
                    } else {
                        servo3.setPower(0);
                        feeder = 0;
                    }
                    sleep(500);
                }
                if (gamepad1.dpad_up) {
                    if (gamepad1.right_bumper) {
                        servo4.setDirection(DcMotorSimple.Direction.REVERSE);
                    } else {
                        servo4.setDirection(DcMotorSimple.Direction.FORWARD);
                    }
                    servo4.setPower(1);
                    claw = 1;
                } else {
                    servo4.setPower(0);
                    claw = 0;
                }
                if (gamepad1.dpad_down) {
                    if (gamepad1.right_bumper) {
                        servo5.setDirection(DcMotorSimple.Direction.REVERSE);
                    } else {
                        servo5.setDirection(DcMotorSimple.Direction.FORWARD);
                    }
                    servo5.setPower(1);
                    drawbridge = 1;
                } else {
                    servo5.setPower(0);
                    drawbridge = 0;
                }
        /*if (gamepad1.b == true && auto == 0) {
          auto = 1;
          Motor1.setTargetPosition(3000);
          Motor2.setTargetPosition(3000);
          Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          ((DcMotorEx) Motor1).setVelocityPIDFCoefficients(0.0075, 0.000075, 0.005, 40);
          ((DcMotorEx) Motor2).setVelocityPIDFCoefficients(0.0075, 0.000075, 0.005, 40);
          Motor1.setPower(1);
          Motor2.setPower(1);
        }
        if (auto == 1 && !Motor1.isBusy() && !Motor2.isBusy()) {
          auto = 0;
          Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }*/
                if (auto == 0) {
                    motor1.setPower(LeftY);
                    motor2.setPower(RightY);
                }
                motor3.setVelocity(1820 * gamepad1.left_trigger);
                motor4.setVelocity(1820 * gamepad1.left_trigger);
                if (gamepad1.left_trigger != 0) {
                    launcher = 1;
                } else {
                    launcher = 0;
                }
                //telemetry.addData("auto", auto);
                telemetry.addData("launcher", launcher);
                telemetry.addData("pickup", pickup);
                telemetry.addData("middle", middle);
                telemetry.addData("feeder", feeder);
                telemetry.addData("claw", claw);
                telemetry.addData("drawbridge", drawbridge);
                telemetry.addData("left", LeftY);
                telemetry.addData("right", RightY);
                telemetry.addData("Motor1", ((DcMotorEx) motor1).getVelocity());
                telemetry.addData("Motor2", ((DcMotorEx) motor2).getVelocity());
                telemetry.addData("Motor1Pos", motor1.getCurrentPosition());
                telemetry.addData("Motor2Pos", motor2.getCurrentPosition());
                telemetry.addData("Motor3", motor3.getVelocity());
                telemetry.addData("Motor4", motor4.getVelocity());
                telemetry.update();
            }
        }
    }
}