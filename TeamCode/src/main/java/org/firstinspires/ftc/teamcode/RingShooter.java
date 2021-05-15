/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Ring Shooter", group = "Iterative Opmode")
//@Disabled
public class RingShooter extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();





    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initializing...");
        telemetry.update();



    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Touch Bottom Trigered",robot.touchBottom.isPressed());
        telemetry.addData("Touch Top Trigered",robot.touchTop.isPressed());
        telemetry.addData("Distance (Meters)","Left %.2f, Right %.2f",robot.distanceLeft);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        telemetry.addData("Status", "Running");
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double left = robot.leftShooter.getVelocity()/robot.maxShootVelocity*100;
        double right = robot.rightShooter.getVelocity()/robot.maxShootVelocity*100;
        // Setup a variable for each drive wheel to save power level for telemetry
        robot.leftShooter.setVelocity(robot.shootVelocity);
        robot.rightShooter.setVelocity(robot.shootVelocity);
        robot.rampTop.setPower(robot.servoPower);
        robot.rampMiddle.setPower(robot.servoPower);
        robot.rampBottom.setPower(robot.servoPower);
        telemetry.addData("Shoot Velocity","Left %.1f Percent, Right %.1f Percent",left,right);
        telemetry.addData("Raw Shoot Velocity","Left %.1f, Right %.1f",robot.leftShooter.getVelocity(),robot.rightShooter.getVelocity());
        telemetry.addData("Battery Voltage","%.1f", robot.voltageSensor.getVoltage());


    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Status", "Stopping...");
        telemetry.update();
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftShooter.setPower(0);
        robot.leftShooter.setPower(0);
        robot.rampBottom.setPower(0);
        robot.rampBottom.setPower(0);
        robot.rampBottom.setPower(0);
        robot.clawArm.setPower(0);
        robot.clawHand.setPower(0);
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

}
