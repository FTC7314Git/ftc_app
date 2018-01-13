package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_LandingGear2 implements AutonomousOp.StepInterface {

    private final Robot.RobotLiftPositionEnum robotLiftPosition;

    private Robot robot;

    private boolean motorAssignedPosition_DoneFlag = false;


    // Constructor, called to create an instance of this class.
    public Step_LandingGear2(Robot.RobotLiftPositionEnum robotLiftPosition) {

        this.robotLiftPosition = robotLiftPosition;
    }


    @Override
    public String getLogKey() {
        return "Step_LandingGear";
    }


    @Override
    public void runStep() {


        if (robot.isStillWaiting()) {
            return;
        }

        robot.motorRobotLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        assignPositionToMotorRobotLift_onlyRunsOnce();

    }


    private void assignPositionToMotorRobotLift_onlyRunsOnce() {

        if (motorAssignedPosition_DoneFlag) {
            return;
        }

        if (Robot.RobotLiftPositionEnum.UP.equals(robotLiftPosition)) {
            robot.motorRobotLift.setPower(Robot.MotorPowerEnum.Med.getValue());


        } else {
            robot.motorRobotLift.setPower(-Robot.MotorPowerEnum.Med.getValue());
        }

        robot.setTimeDelay(1500);
        motorAssignedPosition_DoneFlag = true;


    }


    @Override
    public boolean isStepDone() {


        if (robot.isStillWaiting() || !motorAssignedPosition_DoneFlag) {
            return false;
        }

        robot.motorRobotLift.setPower(0);
        Log.i(getLogKey(), "Step is Done:");
        return true;

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
