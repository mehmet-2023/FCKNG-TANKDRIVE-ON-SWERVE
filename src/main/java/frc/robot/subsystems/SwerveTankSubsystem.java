package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveTankSubsystem extends SubsystemBase {
    private final SparkMax flDrive = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax frDrive = new SparkMax(3, MotorType.kBrushless);
    private final SparkMax blDrive = new SparkMax(7, MotorType.kBrushless);
    private final SparkMax brDrive = new SparkMax(5, MotorType.kBrushless);

    private final SparkMax flSteer = new SparkMax(2, MotorType.kBrushless);
    private final SparkMax frSteer = new SparkMax(4, MotorType.kBrushless);
    private final SparkMax blSteer = new SparkMax(8, MotorType.kBrushless);
    private final SparkMax brSteer = new SparkMax(6, MotorType.kBrushless);

    // Encoders
    private final RelativeEncoder flSteerEncoder = flSteer.getEncoder();
    private final RelativeEncoder frSteerEncoder = frSteer.getEncoder();
    private final RelativeEncoder blSteerEncoder = blSteer.getEncoder();
    private final RelativeEncoder brSteerEncoder = brSteer.getEncoder();

    // PID Controllers
    private final PIDController flSteerPID = new PIDController(0.2, 0, 0);
    private final PIDController frSteerPID = new PIDController(0.2, 0, 0);
    private final PIDController blSteerPID = new PIDController(0.2, 0, 0);
    private final PIDController brSteerPID = new PIDController(0.2, 0, 0);

    // hedef açılar (şu an hepsi 0)
    private final double targetAngle = 0.0;

    public SwerveTankSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.inverted(false);

        flDrive.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        frDrive.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        blDrive.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        brDrive.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        flSteer.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        frSteer.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        blSteer.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        brSteer.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        // PID continuous input (örneğin -π, π arası açı için)
        flSteerPID.enableContinuousInput(-Math.PI, Math.PI);
        frSteerPID.enableContinuousInput(-Math.PI, Math.PI);
        blSteerPID.enableContinuousInput(-Math.PI, Math.PI);
        brSteerPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setLeft(double power) {
        flDrive.set(power);
        blDrive.set(power);
    }

    public void setRight(double power) {
        frDrive.set(power);
        brDrive.set(power);
    }

    public void stop() {
        setLeft(0);
        setRight(0);
    }

    @Override
    public void periodic() {
        double flOutput = flSteerPID.calculate(flSteerEncoder.getPosition(), targetAngle);
        double frOutput = frSteerPID.calculate(frSteerEncoder.getPosition(), targetAngle);
        double blOutput = blSteerPID.calculate(blSteerEncoder.getPosition(), targetAngle);
        double brOutput = brSteerPID.calculate(brSteerEncoder.getPosition(), targetAngle);

        flSteer.set(flOutput);
        frSteer.set(frOutput);
        blSteer.set(blOutput);
        brSteer.set(brOutput);
    }
}
