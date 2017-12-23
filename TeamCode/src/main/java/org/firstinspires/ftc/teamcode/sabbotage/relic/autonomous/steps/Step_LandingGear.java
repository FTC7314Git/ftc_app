package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_LandingGear implements AutonomousOp.StepInterface {

    private Integer distanceEncoderCounts;
    private Robot.MotorPowerEnum motorPowerEnum = Robot.MotorPowerEnum.Med;
    private final Robot.RobotLiftPositionEnum robotLiftPosition;

    private Robot robot;

    private boolean resetMotors_DoneFlag = false;
    private boolean initializedMotors_DoneFlag = false;

    private final int DONE_TOLERANCE = 100;


    private static final double MOTOR_POWER_BALANCE_FACTOR = 1.0;

    private int LIFT_UP_TARGET_POSITION;
    private int LIFT_DOWN_TARGET_POSITION;

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

        logEncoders("run");

        if (robot.isStillWaiting()) {
            return;
        }

        if (robot.isStillWaiting()) {
            return;
        }

        if(Robot.RobotLiftPositionEnum.UP.equals(robotLiftPosition)) {
            robot.motorRobotLift.setTargetPosition(LIFT_UP_TARGET_POSITION);
        } else {
            robot.motorRobotLift.setTargetPosition(LIFT_DOWN_TARGET_POSITION);
        }

    }


    @Override
    public boolean isStepDone() {

        if (robot.isStillWaiting() || !resetMotors_DoneFlag || !initializedMotors_DoneFlag) {
            return false;
        }


        if (isDistanceDone()) {

            logIt("Step is Done:");

            logEncoders(">done");
            robot.resetEncodersAndStopMotors();

            robot.motorRobotLift.setPower(0);
            logEncoders("<done");
            return true;
        }

        return false;
    }

    private int getRemainingDistance() {

        DcMotor encoderMotor = robot.motorRobotLift;

        return Math.abs(distanceEncoderCounts - encoderMotor.getCurrentPosition());

    }

    private boolean isDistanceDone() {

        return Math.abs(getRemainingDistance()) <= DONE_TOLERANCE;
    }

    private void logEncoders(String keySuffix) {

        String key = getLogKey() + keySuffix;

        Log.i(key, "--------------------------------------------------------------------------------------");
        Log.i(key, " CurrentPosition: ROBOTLIFT:" + robot.motorRobotLift.getCurrentPosition());
        Log.i(key, "--------------------------------------------------------------------------------------");
    }

    private void logIt(String methodName) {

        StringBuilder sb = new StringBuilder();
        sb.append(methodName);
        sb.append(" CurrentPosition:" + robot.motorDriveLeft.getCurrentPosition());
        sb.append(" Target:" + robot.motorDriveLeft.getTargetPosition());
        sb.append(" Remaining:" + this.getRemainingDistance());
        Log.i(getLogKey(), sb.toString());

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
