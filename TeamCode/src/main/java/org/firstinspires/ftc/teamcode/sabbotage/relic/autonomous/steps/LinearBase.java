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

package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;


public class LinearBase extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot = new Robot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "robot.init");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        executePlan();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    protected void executePlan() {


    }


    protected void runStepUntilDone(StepInterface step) {

        runStepUntilDone(step, 10000L);

    }


    protected void runStepUntilDone(StepInterface step, Long maxTimeMs) {

        Long timeOutMs = System.currentTimeMillis() + maxTimeMs;

        step.setRobot(this.robot);
        Log.i(step.getLogKey(), "runStepUntilDone: " + step.getLogKey());

        Log.i(step.getLogKey(), "opModeIsActive:" + opModeIsActive());
        Log.i(step.getLogKey(), "Not isStepDone:" + !step.isStepDone());
        Log.i(step.getLogKey(), "timeOut:" + (timeOutMs > System.currentTimeMillis()));


        while (opModeIsActive() && !step.isStepDone() && timeOutMs > System.currentTimeMillis()) {

            Log.i(step.getLogKey(), "step.runStep: " + step.getLogKey());
            step.runStep();
            // Display it for the driver.
            telemetry.addData("Path1", "Running Step: " + step.getLogKey());
            telemetry.update();
        }

        sleep(250);   // optional pause after each step
    }

}
