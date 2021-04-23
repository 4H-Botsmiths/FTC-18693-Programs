package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Autonomous", group = "ForRobot")
public class AutonomusCode extends LinearOpMode {

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

    private boolean CheckColour(String Hex,String Colour) {
        boolean ColourCheck;
        int i;
        double R;
        double G;
        double B;
        List<java.io.Serializable> HexList = null;

        switch (Colour) {
            case "Red":
                Hex = "FF0000";
                break;
            case "Blue":
                Hex = "0000FF";
                break;
            case "White":
                Hex = "FFFFFF";
                break;
        }
        if (Hex.length() == 6) {
            for (i = 1; i <= 6; i++) {
                switch (JavaUtil.inTextGetLetter(Hex, JavaUtil.AtMode.FROM_START, (i - 1))) {
                    case "A":
                        HexList.add(10);
                        break;
                    case "B":
                        HexList.add(11);
                        break;
                    case "C":
                        HexList.add(12);
                        break;
                    case "D":
                        HexList.add(13);
                        break;
                    case "E":
                        HexList.add(14);
                        break;
                    case "F":
                        HexList.add(15);
                        break;
                    default:
                        HexList.add(JavaUtil.inTextGetLetter(Hex, JavaUtil.AtMode.FROM_START, (i - 1)));
                        break;
                }
            }
            R = (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 0, false) * 16 + (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 1, false);
            G = (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 2, false) * 16 + (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 3, false);
            B = (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 4, false) * 16 + (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 5, false);
            Red = Math.round(Color_0.red() * (255 / RedFactor));
            Green = Math.round(Color_0.green() * (255 / GreenFactor));
            Blue = Math.round(Color_0.blue() * (255 / Blue_Factor));
            ColourCheck = Within_Range(10, R, Red) && Within_Range(10, G, Green) && Within_Range(10, B, Blue);
        } else {
            ColourCheck = true;
        }
        return ColourCheck;
    }

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
     * Describe this function...
     */
    private void Move(int in, boolean forward_, boolean moveoverride) {
        while (opModeIsActive()) {
            telemetry.addData("Move Waiting", counter);
            Update_Telemetry();
            counter += 1;
            if (forward_) {
                Motor_0.setDirection(DcMotorSimple.Direction.REVERSE);
                Motor_1.setDirection(DcMotorSimple.Direction.FORWARD);
            } else {
                Motor_0.setDirection(DcMotorSimple.Direction.FORWARD);
                Motor_1.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (moveoverride || !Motor_1.isBusy()) {
                counter = 0;
                Motor_0.setPower(0);
                Motor_1.setPower(0);
                Motor_0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Motor_0.setTargetPosition(in * mm_in * mm_tick);
                Motor_1.setTargetPosition(in * mm_in * mm_tick);
                Motor_0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // set F to 54.1
                ((DcMotorEx) Motor_0).setVelocityPIDFCoefficients(0.0075, 0.000075, 0.005, 54.1);
                ((DcMotorEx) Motor_1).setVelocityPIDFCoefficients(0.0075, 0.000075, 0.005, 54.1);
                Motor_0.setPower(1);
                Motor_1.setPower(1);
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
    }

    /**
     * Describe this function...
     */
    private void Update_Telemetry() {
        telemetry.addData("Motor_0 Target Position", Motor_0.getTargetPosition());
        telemetry.addData("Motor_1 Target Position", Motor_1.getTargetPosition());
        telemetry.addData("Motor_0 Encoder Value", Motor_0.getCurrentPosition());
        telemetry.addData("Motor_1 Encoder Value", Motor_1.getCurrentPosition());
        telemetry.addData("Motor_0 mm/Second", ((DcMotorEx) Motor_0).getVelocity() * mm_tick);
        telemetry.addData("Motor_1 mm/Second", ((DcMotorEx) Motor_0).getVelocity() * mm_tick);
        telemetry.addData("Motor_0 Power", Motor_0.getPower());
        telemetry.addData("Motor_1 Power", Motor_1.getPower());
        telemetry.addData("Motor_0 Direction", Motor_0.getDirection());
        telemetry.addData("Motor_1 Direction", Motor_1.getDirection());
        telemetry.addData("Motor_0 Working?", Motor_0.isBusy());
        telemetry.addData("Motor_1 Working?", Motor_0.isBusy());
        telemetry.update();
    }

    /**
     * Checks if the RANGE VARIABLE is within the range RANGE of the RANGE COMPARISON
     */
    private boolean Within_Range(double range, double range_variable, float range_comparison) {
        boolean range_output;

        range_output = range_comparison < range_variable + range && range_comparison > range_variable - range;
        return range_output;
    }
}
