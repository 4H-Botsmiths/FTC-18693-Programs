package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@TeleOp(name="TankDrive", group="ForRobot")

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
        Motor1.setPower(gamepad1.left_stick_y, MotorTicks);
        Motor2.setPower(gamepad1.right_stick_y, MotorTicks);
        //why did you put the motor ticks in the power? - william
        telemetry.addData("left", gamepad1.left_stick_y*100);
        telemetry.addData("right", gamepad2.left_stick_y*100);
        telemetry.addData("Motor1", ((DcMotorEx)Motor1).getVelocity());
        telemetry.addData("Motor2", ((DcMotorEx)Motor2).getVelocity());
        telemetry.addData("Motor1Pos", Motor1.getCurrentPosition());
        telemetry.addData("Motor2Pos", Motor2.getCurrentPosition());
        telemetry.update();
      }
    }
  }
}