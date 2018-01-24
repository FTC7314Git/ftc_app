package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_PaddleControl implements AutonomousOp.StepInterface {


    private Robot robot;
    private Robot.PaddlePosition paddlePosition;

    boolean resetBlockLiftDoneFlag;

    // Constructor, called to create an instance of this class.
    public Step_PaddleControl(Robot.PaddlePosition paddlePosition) {

        this.paddlePosition = paddlePosition;
    }


    @Override
    public String getLogKey() {
        return "Step_PaddleControl";
    }


    @Override
    public void runStep() {

        init_ResetBlockLift_onlyRunsOnce();

        if (Robot.PaddlePosition.CLOSE.equals(this.paddlePosition)) {

            robot.servoLeftPaddle.setPosition(Robot.SERVO_PADDLE_LEFT_CLOSE);
            robot.servoRightPaddle.setPosition(Robot.SERVO_PADDLE_LEFT_CLOSE);
        }

        if (Robot.PaddlePosition.OPEN.equals(this.paddlePosition)) {

            robot.servoLeftPaddle.setPosition(Robot.SERVO_PADDLE_RIGHT_OPEN);
            robot.servoRightPaddle.setPosition(Robot.SERVO_PADDLE_RIGHT_OPEN);
        }


    }

    private void init_ResetBlockLift_onlyRunsOnce() {

        if (robot.isStillWaiting()) return;

        if (resetBlockLiftDoneFlag == false) {
            robot.motorBlockLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            resetBlockLiftDoneFlag = true;
            robot.setTimeDelay(1000);
            Log.i(getLogKey(), "init_ResetBlockLift_onlyRunsOnce()");

        }
    }




    @Override
    public boolean isStepDone() {


        if (robot.isStillWaiting()) {
            return false;
        }

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
