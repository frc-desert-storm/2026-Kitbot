package frc.robot.subsystems.beegshoot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final SparkMax ShootyMotor;
  private final SparkMax FeedyMotor;
  private final PIDController ShootyPID = new PIDController(0.0001, 0.0, 0.0);
  private final PIDController FeedyPID = new PIDController(0.0001, 0.0, 0.0);

  public void SetFeedySpeed(double FeedyRPM) {
    FeedyPID.setSetpoint(FeedyRPM);
  }

  public void SetShootySpeed(double ShootyRPM) {
    ShootyPID.setSetpoint(ShootyRPM);
  }

  public Shooter() {
    FeedyMotor = new SparkMax(ShootConstants.FeedyCanId, SparkLowLevel.MotorType.kBrushless);
    ShootyMotor = new SparkMax(ShootConstants.ShootyCanId, SparkLowLevel.MotorType.kBrushless);

    var config = new SparkMaxConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kCoast)
        .voltageCompensation(11.5)
        .smartCurrentLimit(80);
    ShootyMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FeedyMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    var ShootyPIDOutput = ShootyPID.calculate(ShootyMotor.getEncoder().getVelocity());
    var FeedyyPIDOutput = FeedyPID.calculate(FeedyMotor.getEncoder().getVelocity());
    ShootyMotor.set(
        ShootyPIDOutput
            + 0.4 * Math.signum(ShootyPID.getSetpoint())
            + 0.0 * ShootyPID.getSetpoint());
    FeedyMotor.set(
        FeedyyPIDOutput
            + 0.35 * Math.signum(FeedyPID.getSetpoint())
            + 0.0 * FeedyPID.getSetpoint());
    Logger.recordOutput("Shooter/ShootMotorSpeed", ShootyMotor.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/FeedyMotorSpeed", FeedyMotor.getEncoder().getVelocity());
  }
}
