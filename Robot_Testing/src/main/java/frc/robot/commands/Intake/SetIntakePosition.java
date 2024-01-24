package frc.robot.commands.Intake;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class SetIntakePosition extends Command {
	private double intakePosition = 0;

	public SetIntakePosition(double intakePosition) {
		this.intakePosition = intakePosition;

		addRequirements(RobotContainer.intakeWrist);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		RobotContainer.intakeWrist.setTargetPosition(intakePosition);
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		RobotContainer.intakeWrist.motionMagicControl();
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
			return RobotContainer.intakeWrist.isInPosition(intakePosition);
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

	}
}
