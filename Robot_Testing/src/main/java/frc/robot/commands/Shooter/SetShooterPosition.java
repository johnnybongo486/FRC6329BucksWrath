package frc.robot.commands.Shooter;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class SetShooterPosition extends Command {
	private int shooterPosition = 0;

	public SetShooterPosition(int shooterPosition) {
		this.shooterPosition = shooterPosition;

		addRequirements(RobotContainer.shooterWrist);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		RobotContainer.shooterWrist.setTargetPosition(shooterPosition);
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		RobotContainer.shooterWrist.motionMagicControl();
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
			return RobotContainer.shooterWrist.isInPosition(shooterPosition);
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

	}
}
