package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

@TeleOp(name="TankDriveTeleop", group="ForRobot")

public class TankDriveTeleop extends LinearOpMode {
  private DcMotor Motor1;
  private DcMotor Motor2;
  private DcMotor Motor3;
  private DcMotor Motor4;
  private CRServo Servo1;
  private CRServo Servo2;
  private CRServo Servo3;
  private CRServo Servo4;
  private CRServo Servo5;
  @Override
  public void runOpMode() {
    Motor1 = hardwareMap.get(DcMotor.class, "Motor_0");
    Motor2 = hardwareMap.get(DcMotor.class, "Motor_1");
    Motor3 = hardwareMap.get(DcMotor.class, "Motor_2");
    Motor4 = hardwareMap.get(DcMotor.class, "Motor_3");
    Servo1 = hardwareMap.get(CRServo.class, "Servo_0");
    Servo2 = hardwareMap.get(CRServo.class, "Servo_1");
    Servo3 = hardwareMap.get(CRServo.class, "Servo_2");
    Servo4 = hardwareMap.get(CRServo.class, "Servo_3");
    Servo5 = hardwareMap.get(CRServo.class, "Servo_4");
    //Motor1.setTargetPosition(0);
    //Motor2.setTargetPosition(0);
    //Motor3.setTargetPosition(0);
    //Motor4.setTargetPosition(0);
    Motor1.setDirection(DcMotorSimple.Direction.REVERSE);
    Motor3.setDirection(DcMotorSimple.Direction.REVERSE);
    Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    double launcher = 0;
    double pickup = 0;
    double middle = 0;
    double feeder = 0;
    double twist = 0;
    double drawbridge = 0;
    double auto = 0;
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        double LeftY = Math.pow(-gamepad1.left_stick_y, 3);
        double RightY = Math.pow(-gamepad1.right_stick_y, 3);
        if (gamepad1.a == true) if (launcher == 0) {
          ((DcMotorEx) Motor3).setVelocity(1820);
          ((DcMotorEx) Motor4).setVelocity(1820);
          launcher = 1;
          sleep(200);
        } else {
          ((DcMotorEx) Motor3).setVelocity(0);
          ((DcMotorEx) Motor4).setVelocity(0);
          launcher = 0;
          sleep(200);
        }
        if (gamepad1.b == true) if (pickup == 0) {
          if (gamepad1.right_bumper == true) {
            Servo1.setDirection(DcMotorSimple.Direction.REVERSE);
          } else {
            Servo1.setDirection(DcMotorSimple.Direction.FORWARD);
          }
          Servo1.setPower(1);
          pickup = 1;
          sleep(100);
        } else {
          Servo1.setPower(0);
          pickup = 0;
          sleep(100);
        }
        if (gamepad1.y == true) if (middle == 0) {
          if (gamepad1.right_bumper == true) {
            Servo2.setDirection(DcMotorSimple.Direction.REVERSE);
          } else {
            Servo2.setDirection(DcMotorSimple.Direction.FORWARD);
          }
          Servo2.setPower(1);
          middle = 1;
          sleep(100);
        } else {
          Servo2.setPower(0);
          middle = 0;
          sleep(100);
        }
        if (gamepad1.x == true) if (feeder == 0) {
          if (gamepad1.right_bumper == true) {
            Servo3.setDirection(DcMotorSimple.Direction.REVERSE);
          } else {
            Servo3.setDirection(DcMotorSimple.Direction.FORWARD);
          }
          Servo3.setPower(1);
          feeder = 1;
          sleep(100);
        } else {
          Servo3.setPower(0);
          feeder = 0;
          sleep(100);
        }
        if (gamepad1.dpad_up == true) if (twist == 0) {
          if (gamepad1.right_bumper == true) {
            Servo4.setDirection(DcMotorSimple.Direction.REVERSE);
          } else {
            Servo4.setDirection(DcMotorSimple.Direction.FORWARD);
          }
          Servo4.setPower(1);
          twist = 1;
          sleep(100);
        } else {
          Servo4.setPower(0);
          twist = 0;
          sleep(100);
        }
        if (gamepad1.dpad_down == true) if (drawbridge == 0) {
          if (gamepad1.right_bumper == true) {
            Servo5.setDirection(DcMotorSimple.Direction.REVERSE);
          } else {
            Servo5.setDirection(DcMotorSimple.Direction.FORWARD);
          }
          Servo5.setPower(1);
          drawbridge = 1;
          sleep(100);
        } else {
          Servo5.setPower(0);
          drawbridge = 0;
          sleep(100);
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
          Motor1.setPower(LeftY);
          Motor2.setPower(RightY);
        }
        //telemetry.addData("auto", auto);
        telemetry.addData("launcher", launcher);
        telemetry.addData("pickup", pickup);
        telemetry.addData("middle", middle);
        telemetry.addData("feeder", feeder);
        telemetry.addData("left", LeftY);
        telemetry.addData("right", RightY);
        telemetry.addData("Motor1", ((DcMotorEx)Motor1).getVelocity());
        telemetry.addData("Motor2", ((DcMotorEx)Motor2).getVelocity());
        telemetry.addData("Motor1Pos", Motor1.getCurrentPosition());
        telemetry.addData("Motor2Pos", Motor2.getCurrentPosition());
        telemetry.addData("Motor3", ((DcMotorEx)Motor3).getVelocity());
        telemetry.addData("Motor4", ((DcMotorEx)Motor4).getVelocity());
        telemetry.update();
      }
    }
  }
}