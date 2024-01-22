package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class VisionAlign extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private Boolean robotCentricSup;
    private double slowSpeed = 0.2;
    private double midSpeed = 0.5;
    private double elevatorHeight = 0;

    private double tx = 0;
    private double ty = 0;

    private double kPgain = 0.016; /* percent throttle per degree of error */   // 0.016 
    private double kIgain = 0.2;  // .06
    private double kDgain = 0.0002; /* percent throttle per angular velocity dps */
    private double errorSum = 0;
    private double iLimit = 4;
    private double lastTimeStamp = 0;
    private double lastError = 0; 

    public VisionAlign(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, Boolean robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        addRequirements(RobotContainer.frontLimelight);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    public void initialize() {
        tx = RobotContainer.frontLimelight.getX();
        ty = RobotContainer.frontLimelight.getY();
        errorSum = 0;
        lastError = 0;
        lastTimeStamp = Timer.getFPGATimestamp();
    }
    
    @Override
    public void execute() {
        //TODO: I'm not sure if I disabled the elevator consideration here correctly.
        //elevatorHeight = RobotContainer.elevator.getCurrentPosition();

        // find target location
        tx = RobotContainer.frontLimelight.getX();
        ty = RobotContainer.frontLimelight.getY();
        
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

        double dt = Timer.getFPGATimestamp() - lastTimeStamp;
        
        if (Math.abs(tx) < iLimit) {
            errorSum += tx * dt;
        }

        double errorRate = (tx - lastError) / dt;

        double rotationVal = (tx) * kPgain + kIgain * errorSum + errorRate * kDgain;

        /*if (elevatorHeight >= 30000) {
            translationVal = translationVal * slowSpeed;
            strafeVal = strafeVal * slowSpeed;
            rotationVal = rotationVal * slowSpeed;
        }

        else if (elevatorHeight > 5000 && elevatorHeight < 29999) {
            translationVal = translationVal * midSpeed;
            strafeVal = strafeVal * midSpeed;
            rotationVal = rotationVal * midSpeed;
        }

        else {}*/

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup, 
            true
        );

        lastTimeStamp = Timer.getFPGATimestamp();
        lastError = tx;
    }
}
