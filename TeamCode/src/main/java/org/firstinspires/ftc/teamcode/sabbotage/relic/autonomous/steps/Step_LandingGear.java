package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_LandingGear implements AutonomousOp.StepInterface {

    private final Robot.RobotLiftPositionEnum robotLiftPosition;

    private Robot robot;

    private boolean resetMotor_DoneFlag = false;
    private boolean motorAssignedPosition_DoneFlag = false;


    private int LIFT_UP_TARGET_POSITION = 0;
    private int LIFT_DOWN_TARGET_POSITION = 170;

    // Constructor, called to create an instance of this class.
    public Step_LandingGear(Robot.RobotLiftPositionEnum robotLiftPosition) {

        this.robotLiftPosition = robotLiftPosition;
    }


    @Override
    public String getLogKey() {
        return "Step_LandingGear";
    }


    @Override
    public void runStep() {

        Log.i(getLogKey(), "Current position: " + robot.motorRobotLift.getCurrentPosition());

        init_ResetRobotLift_onlyRunsOnce();

        if (robot.isStillWaiting()) {
            return;
        }

        Log.i(getLogKey(), "position: " + robot.motorRobotLift.getCurrentPosition());



        assignPositionToMotorRobotLift_onlyRunsOnce();


    }


    private void assignPositionToMotorRobotLift_onlyRunsOnce() {

        if (motorAssignedPosition_DoneFlag) return;

        robot.motorRobotLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRobotLift.setPower(Robot.MotorPowerEnum.Med.getValue());


        if (Robot.RobotLiftPositionEnum.UP.equals(robotLiftPosition)) {
            robot.motorRobotLift.setTargetPosition(LIFT_UP_TARGET_POSITION);


        } else {
            robot.motorRobotLift.setTargetPosition(LIFT_DOWN_TARGET_POSITION);
        }

        robot.setTimeDelay(1500);
        motorAssignedPosition_DoneFlag = true;


    }

    private void init_ResetRobotLift_onlyRunsOnce() {

        if (robot.isStillWaiting()) return;

        if (resetMotor_DoneFlag == false) {
            robot.motorRobotLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorRobotLift.setDirection(DcMotorSimple.Direction.REVERSE);
            resetMotor_DoneFlag = true;
            robot.setLoopDelay();
            Log.i(getLogKey(), "init_ResetRobotLift_onlyRunsOnce()");
            Log.i(getLogKey(), "init position: " + robot.motorRobotLift.getCurrentPosition());

        }
    }

    @Override
    public boolean isStepDone() {


        if (robot.isStillWaiting() || !resetMotor_DoneFlag || !motorAssignedPosition_DoneFlag) {
            return false;
        }

        Log.i(getLogKey(), "Step is Done:");
        return true;

    }

    private void logEncoders() {


        Log.i(getLogKey(), " CurrentPosition: ROBOT LIFT:" + robot.motorRobotLift.getCurrentPosition());
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
