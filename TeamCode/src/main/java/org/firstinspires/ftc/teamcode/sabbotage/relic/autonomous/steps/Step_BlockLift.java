package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_BlockLift implements AutonomousOp.StepInterface {

    private Robot robot;
    boolean resetBlockLiftDoneFlag;
    private boolean blockLiftDoneFlag = false;


    // Constructor, called to create an instance of this class.
    public Step_BlockLift() {

    }


    // Constructor, called to create an instance of this class.
    public Step_BlockLift(boolean alreadyInitalizeded) {

        this.blockLiftDoneFlag = alreadyInitalizeded;
    }

    @Override
    public String getLogKey() {
        return "Step_BlockLift";
    }


    @Override
    public void runStep() {

        init_ResetBlockLift_onlyRunsOnce();

        raiseControlBlockLift();

    }

    private void init_ResetBlockLift_onlyRunsOnce() {

        if (robot.isStillWaiting()) return;

        if (resetBlockLiftDoneFlag == false) {
            robot.motorBlockLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            resetBlockLiftDoneFlag = true;
            robot.setTimeDelay(300);
            Log.i(getLogKey(), "init_ResetBlockLift_onlyRunsOnce()");

        }
    }


    private void raiseControlBlockLift() {

        if (robot.isStillWaiting()) return;

        int targetBlockLiftPosition = 180;

        robot.motorBlockLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBlockLift.setTargetPosition(targetBlockLiftPosition);
        robot.motorBlockLift.setPower(.6);
        this.blockLiftDoneFlag = true;
        Log.i(getLogKey(), "raiseControlBlockLift encoder" + robot.motorBlockLift.getCurrentPosition());
    }


    @Override
    public boolean isStepDone() {

        if (robot.isStillWaiting()) return false;

        if (this.blockLiftDoneFlag) {

            return true;
        }
        return false;
    }


    @Override
    public boolean isAborted() {
        return false;
    }


    @Override
    public void setRobot(Robot robot) {
        this.robot = robot;
    }


}
