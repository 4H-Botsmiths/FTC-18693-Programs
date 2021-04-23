package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@TeleOp(name = "KoltonsFirstTankDrive", group = "ForRobot")

public class KoltonsFirstTankDrive extends LinearOpMode {
    private DcMotorEx Motor1 = hardwareMap.get(DcMotorEx.class, "Motor_0");
    private DcMotorEx Motor2 = hardwareMap.get(DcMotorEx.class, "Motor_1");
    private DcMotorEx Motor3 = hardwareMap.get(DcMotorEx.class, "Motor_2");
    private DcMotorEx Motor4 = hardwareMap.get(DcMotorEx.class, "Motor_3");
    double MotorTicks = 28;

    @Override
    public void runOpMode() {
        Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double MotorLeft = Math.pow(-gamepad1.left_stick_y, 3);
                double MotorRight = Math.pow(-gamepad1.right_stick_y, 3);
                Motor1.setPower(MotorLeft);
                Motor2.setPower(MotorRight);
                Motor3.setVelocity();
                Motor4.setVelocity();
            }
        }
    }
}