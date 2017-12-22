package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;

public class Step_TurnLeft extends Step_TurnRight {

    // Constructor, called to create an instance of this class.
    public Step_TurnLeft(double angleDegrees) {
        super(angleDegrees);
    }


    @Override

    public String getLogKey() {
        return "Step_TurnLeft";
    }


    @Override
    protected double remainingAngle() {


        return Math.abs(this.targetAngle - robot.getAngle());
    }


    @Override
    protected double determinePower() {

        return -super.determinePower();

    }

    @Override
    protected boolean hasOverShotTargetAngle() {
        return false;
    }

}
