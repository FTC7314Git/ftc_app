package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;

public class Step_TurnRight extends Step_TurnLeft {

    // Constructor, called to create an instance of this class.
    public Step_TurnRight(double angleDegrees) {
        super(angleDegrees);
    }


    @Override

    public String getLogKey() {
        return "Step_TurnRight";
    }


    @Override
    protected double remainingAngle() {


        return robot.getAngle() - this.targetAngle;
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
