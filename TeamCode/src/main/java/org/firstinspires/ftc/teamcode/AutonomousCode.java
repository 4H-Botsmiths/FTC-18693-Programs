package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.Serializable;
import java.util.List;

@Autonomous(name = "Autonomous", group = "ForRobot")
public class AutonomousCode extends LinearOpMode {

    private ColorSensor Color_0;
    private BNO055IMU imu;
    private DcMotor Motor_0;
    private DcMotor Motor_1;

    float Red;
    float Green;
    float Blue;
    double counter;
    double RedFactor;
    double GreenFactor;
    double Blue_Factor;
    int mm_tick;
    int mm_in;


    /**
     * Describe this function...
     */
    private void Update_Color() {
        Red = Math.round(Color_0.red() * (255 / RedFactor));
        Green = Math.round(Color_0.green() * (255 / GreenFactor));
        Blue = Math.round(Color_0.blue() * (255 / Blue_Factor));
    }

    /**
     * Describe this function...
     */
    private void Turn(double degrees, boolean turnoverride) {
        BNO055IMU.Parameters imuPar;
        Orientation angles = null;

        Update_Telemetry();
        imuPar = new BNO055IMU.Parameters();
        imuPar.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuPar.loggingEnabled = false;
        imu.initialize(imuPar);
        Motor_0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
            telemetry.addData("Turn Waiting", counter);
            Update_Telemetry();
            counter += 1;
            if (turnoverride || !Motor_1.isBusy()) {
                degrees = degrees * -1;
                counter = 0;
                if (degrees > 0) {
                    Motor_0.setDirection(DcMotorSimple.Direction.FORWARD);
                    Motor_1.setDirection(DcMotorSimple.Direction.FORWARD);
                } else {
                    Motor_0.setDirection(DcMotorSimple.Direction.REVERSE);
                    Motor_1.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                Motor_0.setPower(1);
                Motor_1.setPower(1);
                while (!(Within_Range(1, degrees, angles.firstAngle) || isStopRequested())) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("angle", angles.firstAngle);
                    telemetry.addData("target", degrees);
                    telemetry.addData("done?", Within_Range(1, degrees, angles.firstAngle) || isStopRequested());
                    Update_Telemetry();
                }
                Motor_0.setPower(0);
                Motor_1.setPower(0);
                break;
            }
        }
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int Ticks_Rotation;
        int Wheel_Circumference;

        Color_0 = hardwareMap.get(ColorSensor.class, "Color_0");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        Motor_0 = hardwareMap.get(DcMotor.class, "Motor_0");
        Motor_1 = hardwareMap.get(DcMotor.class, "Motor_1");

        telemetry.addData("Status", "Please wait");
        telemetry.update();
        Motor_0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Ticks_Rotation = 288;
        Wheel_Circumference = (int) 282.6;
        mm_tick = Wheel_Circumference / Ticks_Rotation;
        mm_in = (int) 25.4;
        counter = 0;
        RedFactor = 338 * 1;
        GreenFactor = 559 * 1;
        Blue_Factor = 494 * 1;
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            Move(48, true, false);
            Turn(90, false);
            Move(24, true, false);
            Turn(-90, false);
            while (opModeIsActive()) {
                Update_Telemetry();
            }
        }
    };

