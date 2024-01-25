package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class IntakeRunFeeder extends Command {

	public boolean beamBreak;
    
    public IntakeRunFeeder() {
        addRequirements(RobotContainer.feeder);
    }
	// Called just before this Command runs the first time
	public void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		//read sensor
		beamBreak = RobotContainer.feeder.readInput();
		
		//stop feeder if beam broken
		if (beamBreak == false) {
			RobotContainer.feeder.setSpeed(0.0);
		}

		else {
			RobotContainer.feeder.setSpeed(0.6);
		}
        
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
		return beamBreak == false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}


