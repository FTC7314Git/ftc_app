package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;

import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_TurnRight extends Step_TurnLeft {

    // Constructor, called to create an instance of this class.
    public Step_TurnRight(double angleDegrees) {
        super(angleDegrees);
    }
    // Constructor, called to create an instance of this class.
    public Step_TurnRight(Robot.RobotStartPositionEnum robotStartPositionEnum) {

        super(robotStartPositionEnum);
    }

    @Override

    public String getLogKey() {
        return "Step_TurnRight";
    }


    @Override
    protected double remainingAngle() {


        return  this.targetAngle - robot.getAngle();
    }


    @Override
    protected double determinePower() {

        return -super.determinePower();

    }

    @Override
    protected boolean hasOverShotTargetAngle() {
        return remainingAngle() < 0;
    }

}
