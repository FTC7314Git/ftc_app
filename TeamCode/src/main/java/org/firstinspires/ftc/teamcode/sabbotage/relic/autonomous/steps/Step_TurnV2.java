package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;


import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_TurnV2 implements AutonomousOp.StepInterface, StepInterface {

    private static final double TARGET_TOLERANCE = .5;

    private Robot.MotorPowerEnum motorPowerEnum = Robot.MotorPowerEnum.Med;

    protected Robot robot;

    private boolean resetMotors_DoneFlag = false;
    protected Double targetAngle = 0.0;


    // Constructor, called to create an instance of this class.
    public Step_TurnV2(Double targetAngle) {
        this.targetAngle = targetAngle;
    }

    @Override
    public String getLogKey() {
        return "Step_Turn";
    }


    @Override
    public void runStep() {

        resetMotorsAndInitializeDirection_Only_Once();

        if (robot.isStillWaiting()) {
            return;
        }

        // We should not need to do this, but have see when motor(s) have not been reset.
        robot.runWithEncoders_MAINTAINS_SPEED();

        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        turn();

    }


    private boolean isAtTargetAngle() {

        return Math.abs(remainingAngle()) <= TARGET_TOLERANCE;
    }

    protected double remainingAngle() {

        return this.targetAngle - getAdjustedAngle();

    }


    private double getAdjustedAngle() {

        return robot.getAngleOffZero();

    }

    private void turn() {


        double power = .03d;

        if (getAdjustedAngle() > targetAngle) {

            robot.motorDriveRight.setPower(-power);
            robot.motorDriveLeft.setPower(+power);
            logIt("turnRight:");

        } else {

            robot.motorDriveRight.setPower(+power);
            robot.motorDriveLeft.setPower(-power);
            logIt("TurningLeft:");
        }
    }


    private void resetMotorsAndInitializeDirection_Only_Once() {

        if (resetMotors_DoneFlag == false) {

            robot.runWithoutEncoders();
            robot.setDriveMotorForwardDirection();

            resetMotors_DoneFlag = true;
            robot.setLoopDelay();
        }

    }


    @Override
    public boolean isStepDone() {


        if (robot.isStillWaiting() || resetMotors_DoneFlag == false) {
            return false;

        }


        if (isAtTargetAngle()) {

            Log.i(getLogKey(), "Step is Done at angle:" + robot.getAngle());

            robot.motorDriveRight.setPower(0);
            robot.motorDriveLeft.setPower(0);
            return true;
        }


        return false;
    }


    private void logIt(String methodName) {

        StringBuilder sb = new StringBuilder();
        sb.append(" , CurrentAngle:" + robot.getAngle());
        sb.append(" , TargetAngle:" + targetAngle);
        sb.append(" , RemainingAngle:" + remainingAngle());
        sb.append(" , L Power:" + robot.motorDriveLeft.getPower() + " , R Power:" + robot.motorDriveRight.getPower());

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
