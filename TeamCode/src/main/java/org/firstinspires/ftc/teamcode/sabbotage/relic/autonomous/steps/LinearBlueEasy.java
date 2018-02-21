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


import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

@Autonomous(name = "LinearBlueEasy", group = "Blue")
public class LinearBlueEasy extends LinearBase {


    protected void executePlan() {

        runStepUntilDone(new Step_PaddleControl(Robot.PaddlePosition.CLOSE));
//        runStepUntilDone(new Step_JewelScoringV2(Robot.TeamEnum.BLUE));

        robot.motorBlockLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runStepUntilDone(new Step_BlockLift(200));
        robot.setVuMark(RelicRecoveryVuMark.CENTER);
//        runStepUntilDone(new Step_ReadVuMark(), 5000L);
        runStepStraightBasedOnVuMark();

        runStepUntilDone(new Step_TurnV2(-100.0));
        runStepUntilDone(new Step_BlockLift(10));
        runStepUntilDone(new Step_PaddleControl(Robot.PaddlePosition.OPEN));
        runStepUntilDone(new Step_Straight(500, Robot.DirectionEnum.FORWARD), 8000L);
        runStepUntilDone(new Step_Straight(350, Robot.DirectionEnum.REVERSE));

    }

    private void runStepStraightBasedOnVuMark() {

        if (robot.getVuMark() == null) {
            robot.setVuMark(RelicRecoveryVuMark.RIGHT);
        }

        Integer distance;
        switch (robot.getVuMark()) {

            case LEFT:
                distance = 1500;
                break;
            case CENTER:
                distance = 1800;
                break;
            case RIGHT:
                distance = 2200;
                break;
            default:
                distance = 1500;
        }

        Log.i("Step_", "runStepStraightBasedOnVuMark:" + distance);

        runStepUntilDone(new Step_Straight(distance, Robot.DirectionEnum.REVERSE));
    }

}
