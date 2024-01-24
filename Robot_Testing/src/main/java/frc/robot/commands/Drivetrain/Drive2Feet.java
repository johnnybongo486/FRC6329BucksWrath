package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class Drive2Feet extends Command {    
    private Swerve s_Swerve;    
    private Boolean robotCentricSup;

    private double tx = 0;
    private double ta = 0;

    private final PIDController angleController = new PIDController(0.01, 0, 0);  // 0.012 0.2     0.0002
    private double targetAngle = 0;
    private final PIDController distanceController = new PIDController(0.02, 0, 0);
    private double targetArea = 9;  // what is the area when we pick up gp?

    public Drive2Feet(Swerve s_Swerve, Boolean robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        addRequirements(RobotContainer.rearLimelight);

        this.robotCentricSup = robotCentricSup;
    }

    public void initialize() {
        tx = RobotContainer.rearLimelight.getX();
        ta = RobotContainer.rearLimelight.getArea();

        angleController.setTolerance(0.05);  // needs to be checked
        distanceController.setTolerance(0.05);  // needs to be checked
    }
    
    @Override
    public void execute() {

        s_Swerve.drive(
        new Translation2d(-0.2, 0.0).times(Constants.Swerve.maxSpeed), 
        0 * Constants.Swerve.maxAngularVelocity, 
        !robotCentricSup, 
        true
        );  

    }

    public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
        
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
