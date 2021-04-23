package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "KoltonsFirstTankDrive", group = "ForRobot")

public class KoltonsFirstTankDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor motor1 = hardwareMap.get(DcMotor.class, "Motor_0");
        DcMotor motor2 = hardwareMap.get(DcMotor.class, "Motor_1");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double MotorTicks = 28;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double MotorLeft = Math.pow(-gamepad1.left_stick_y, 3)*MotorTicks;
                double MotorRight = Math.pow(-gamepad1.right_stick_y, 3)*MotorTicks;
                ((DcMotorEx) motor1).setVelocity(MotorLeft);
                ((DcMotorEx) motor2).setVelocity(MotorRight);
            }
        }
    }
}