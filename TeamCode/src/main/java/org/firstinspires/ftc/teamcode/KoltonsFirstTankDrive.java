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
    private DcMotor Motor1;
    private DcMotor Motor2;

    @Override
    public void runOpMode() {
        Motor1 = hardwareMap.get(DcMotor.class, "Motor_0");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor_1");
        Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double MotorTicks = 28;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double MotorLeft = Math.pow(-gamepad1.left_stick_y*MotorTicks, 3);
                double MotorRight = Math.pow(-gamepad1.right_stick_y*MotorTicks, 3);
                ((DcMotorEx) Motor1).setVelocity(MotorLeft);
                ((DcMotorEx) Motor2).setVelocity(MotorRight);
            }
        }
    }
}