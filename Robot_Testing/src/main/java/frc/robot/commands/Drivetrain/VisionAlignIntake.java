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


public class VisionAlignIntake extends Command {    
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
    private double ta = 0;

    private double kPgain = 0.016; /* percent throttle per degree of error */   // 0.016 
    private double kIgain = 0.2;  // .06
    private double kDgain = 0.0002; /* percent throttle per angular velocity dps */

    private double aPgain = 0.016; /* percent throttle per degree of error */   // 0.016 
    private double aIgain = 0.2;  // .06
    private double aDgain = 0.0002; /* percent throttle per angular velocity dps */
    private double targetArea = 0;  // what is the area when we pick up gp?

    private double errorSum = 0;
    private double errorSumA = 0;
    private double iLimit = 4;
    private double iLimitA = 4;
    private double lastTimeStamp = 0;
    private double lastError = 0;
    private double lastErrorA = 0; 

    public VisionAlignIntake(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, Boolean robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        addRequirements(RobotContainer.rearLimelight);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    public void initialize() {
        tx = RobotContainer.rearLimelight.getX();
        ty = RobotContainer.rearLimelight.getY();
        ta = RobotContainer.rearLimelight.getArea();
        errorSum = 0;
        errorSumA = 0;
        lastError = 0;
        lastErrorA = 0;
        lastTimeStamp = Timer.getFPGATimestamp();
    }
    
    @Override
    public void execute() {
        //TODO: I'm not sure if I disabled the elevator consideration here correctly.
        //elevatorHeight = RobotContainer.elevator.getCurrentPosition();

        // find target location
        tx = RobotContainer.rearLimelight.getX();
        ty = RobotContainer.rearLimelight.getY();
        ta = RobotContainer.rearLimelight.getArea();

        double dt = Timer.getFPGATimestamp() - lastTimeStamp;
        
        if (Math.abs(tx) < iLimit) {
            errorSum += tx * dt;
        }

        if (Math.abs(ta) < iLimitA) {
            errorSumA += ta *dt;
        }

        double errorRate = (tx - lastError) / dt;
        double errorRateA = (ta - lastErrorA) / dt;

        double rotationVal = (tx) * kPgain + kIgain * errorSum + errorRate * kDgain;
        double strafeVal = (tx) * kPgain + kIgain * errorSum + errorRate * kDgain;
        double translationVal = (ta) * aPgain + aIgain * errorSumA + errorRateA * aDgain;

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
            robotCentricSup, 
            true
        );

        lastTimeStamp = Timer.getFPGATimestamp();
        lastError = tx;
        lastErrorA = ta;
    }
}
