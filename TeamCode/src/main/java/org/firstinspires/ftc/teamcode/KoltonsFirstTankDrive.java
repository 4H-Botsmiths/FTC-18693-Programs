package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@TeleOp(name="TankDriveTeleop", group="ForRobot")

public class TankDrive extends LinearOpMode {
  private DcMotor Motor1;
  private DcMotor Motor2;
 double MotorTicks = 28;
  @Override
  public void runOpMode() {
    Motor1 = hardwareMap.get(DcMotor.class, "Motor_0");
    Motor2= hardwareMap.get(DcMotor.class, "Motor_1");
    Motor1.setDirection(DcMotorSimple.Direction.REVERSE);
    Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        ((DcMotorEx) Motor1).setVelocity(Math.multiplyExact(gamepad1.left_stick_y, MotorTicks));
        ((DcMotorEx) Motor2).setVelocity(Math.multiplyExact(gamepad1.right_stick_y, MotorTicks));
      }
    }
  }
}