package frc.robot.commands.Shooter;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class SetShooterVelocity extends Command {
	private double leftShotVelocity = 0;
    private double rightShotVelocity = 0;

	public SetShooterVelocity(double leftVelocity, double rightVelocity) {
		this.leftShotVelocity = leftVelocity;
        this.rightShotVelocity = rightVelocity;

		addRequirements(RobotContainer.leftShooter);
        addRequirements(RobotContainer.rightShooter);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		RobotContainer.leftShooter.setTargetVelocity(leftShotVelocity);
        RobotContainer.rightShooter.setTargetVelocity(rightShotVelocity);
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		RobotContainer.leftShooter.velocityControl();
        RobotContainer.rightShooter.velocityControl();
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
			return RobotContainer.leftShooter.isAtVelocity(leftShotVelocity) && RobotContainer.rightShooter.isAtVelocity(rightShotVelocity);
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

	}
}
