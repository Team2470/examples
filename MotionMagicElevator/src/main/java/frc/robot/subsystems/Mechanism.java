package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.MechanismConstants;

public class Mechanism extends SubsystemBase {

    private enum ControlMode {
       kHome, kStop, kOpenLoop, kMotionMagic
    }

    //
    // Hardware
    //
    private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    private final TalonFX m_motor = new TalonFX(MechanismConstants.kMotorID);
    private final TalonFX m_motorFollower = new TalonFX(MechanismConstants.kMotorFollowerID);
    private final DigitalInput m_retractLimit = new DigitalInput(MechanismConstants.kRetractLimitSwitchChannel);;

    //
    // State
    //
    private ControlMode m_controlMode = ControlMode.kStop;
    private double m_demand;
    private boolean m_homed;

    private final VoltageOut m_homeVoltageRequst = new VoltageOut(0);
    private final MotionMagicExpoTorqueCurrentFOC m_motionMagicRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    //
    // SysID
    //
    private final VoltageOut m_voltReq = new VoltageOut(0.0);
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            // Use default ramp rate (1 V/s)
            null,
            // Reduce dynamic step voltage to 4 to prevent brownout
            Volts.of(4),
            // Use default timeout (10 s)
            null, 
            // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> m_motor.setControl(m_voltReq.withOutput(volts.in(Volts))),
            null,
            this
        )
    );

    public Mechanism() {
        //
        // Base Motor configuration: Brake Mode, Current limits, etc...
        //
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = 125;

        //
        // Setup Follower motor
        //
        m_motorFollower.getConfigurator().apply(m_motorConfig);
        m_motorFollower.optimizeBusUtilization();
        m_motorFollower.setControl(new Follower(MechanismConstants.kMotorID, true));

        //
        // Apply add extra leader configuration on top of the base config
        // - Any control related settings for PID, Motion Magic, Remote Sensors, etc.. 
        // - Soft limits should only be applied to the leader motor, if they are applied to the follower
        //   it may stop moving if their built in encoders are not in sync.
        //
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 80;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.25;

        // PID Slot 0
        //
        // TODO may to convert gains from Recalc to something in rotations
        // Also note that the gear ratio on the elevator is 20:1, but the effective gear ratio is 10:1 due to ie being cascade 
        // https://www.reca.lc/linear?angle=%7B%22s%22%3A103.582964%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A30%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Kraken%20X60%20%28FOC%29%2A%22%7D&ratio=%7B%22magnitude%22%3A10%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1.751%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A30%2C%22u%22%3A%22in%22%7D
        m_motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        m_motorConfig.Slot0.kS = 0; // TODO: Add 0.25 V output to overcome static friction
        m_motorConfig.Slot0.kV = 0; // TODO: A velocity target of 1 rps results in 0.12 V output
        m_motorConfig.Slot0.kA = 0; // TODO: An acceleration of 1 rps/s requires 0.01 V output
        m_motorConfig.Slot0.kP = 0; // TODO: A position error of 2.5 rotations results in 12 V output
        m_motorConfig.Slot0.kI = 0; // TODO: no output for integrated error
        m_motorConfig.Slot0.kD = 0; // TODO: A velocity error of 1 rps results in 0.1 V output

        // Motion Magic
        m_motorConfig.MotionMagic.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        m_motorConfig.MotionMagic.MotionMagicExpo_kV = 0.0; // kV is around 0.12 V/rps
        m_motorConfig.MotionMagic.MotionMagicExpo_kA = 0.0; // Use a slower kA of 0.1 V/(rps/s)

        m_motor.getConfigurator().apply(m_motorConfig);

        // We want to read position data from the leader motor
        m_motor.getPosition().setUpdateFrequency(50);

        // These 3 are needed for the follower motor to work
        m_motor.getDutyCycle().setUpdateFrequency(50);
        m_motor.getMotorVoltage().setUpdateFrequency(50);
        m_motor.getTorqueCurrent().setUpdateFrequency(50);
        m_motor.optimizeBusUtilization();    


        //
        // Mechanism state
        //
        m_homed = false;
    }

    //
    //
    //

    public boolean isAtRetractLimit() {
        return !m_retractLimit.get();
    }

    public double getHeight() {
        return m_motor.getPosition().getValueAsDouble() * MechanismConstants.kRotationToInches;
    }

    public void stop() {
        m_controlMode = ControlMode.kStop;
        m_demand = 0.0;
    }

    public void setOutputVoltage(double OutputVoltage) {
        m_controlMode = ControlMode.kOpenLoop;
        m_demand = OutputVoltage;
    }

    public void setMotionMagic(double heightInches) {
        m_controlMode = ControlMode.kMotionMagic;
        m_demand = heightInches;
    }

    //
    // Commands
    //

    public Command homeCommand() {
        return new FunctionalCommand(
            () -> {
                // Clear homed state
                m_homed = false;

                // Disable soft limits
                m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
                m_motor.getConfigurator().apply(m_motorConfig);
            },
            () -> {
                // Switch to home control mode
                m_controlMode = ControlMode.kHome;
                m_demand = -2.0;
            },
            (Boolean) -> {
                // Enable soft limits
                m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
                m_motor.getConfigurator().apply(m_motorConfig);

                // Stop the elevator
                stop();
            },
            this::isAtRetractLimit,
            this
        );
    }

    public Command openLoopCommand(DoubleSupplier OutputVoltageSupplier) {
        return Commands.runEnd(
            () -> this.setOutputVoltage(OutputVoltageSupplier.getAsDouble()), this::stop, this);
    }

    public Command openLoopCommand(double OutputVoltage) {
        return openLoopCommand(() -> OutputVoltage);
    }

    public Command motionMagicCommand(DoubleSupplier heightInches) {
        return Commands.runEnd(
            () -> this.setMotionMagic(heightInches.getAsDouble()), this::stop, this);
    }

    public Command motionMagicCommand(double heightInches) {
        return motionMagicCommand(() -> heightInches);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        // TODO add switch to disable break mode when disabled

        SmartDashboard.putString("Mechanism wanted control mode", m_controlMode.toString());
        if (!m_homed && m_controlMode != ControlMode.kHome) {
            // If the elevator is not homed and we are not homing then stop movement
            stop();
        }

        // Elevator is not homed but is at the retract limit, so home the elevator!
        if (!m_homed && isAtRetractLimit()) {
            m_motor.setPosition(0);
            m_homed = true;
        }

        SmartDashboard.putString("Mechanism control mode", m_controlMode.toString());
        SmartDashboard.putNumber("Mechanism demand", m_demand);
        SmartDashboard.putBoolean("Mechanism is at retract limit", isAtRetractLimit());
        SmartDashboard.putBoolean("Mechanism homed", m_homed);
        SmartDashboard.putNumber("Mechanism height", getHeight());
        SmartDashboard.putNumber("Mechanism rotations", m_motor.getPosition().getValueAsDouble());


        // if (!m_homed && m_controlMode == ControlMode.kHome) {
        //     // Elevator is not homed
            
        //     // TODO make sure other mechanisms on elevator (Arm & Wrist) don't move if the elveator isn't homed (or are able to move in a safe direction like toward starting config) 
        //     if (m_controlMode == ControlMode.kOpenLoop && m_demand < 0) {
        //         // We can only allow it to move down in open loop (driver pressing down button).
        //         m_motor.setVoltage(m_demand);
        //     } else {
        //         // Otherwise don't let the elevator move down
        //         m_motor.stopMotor();
        //     }
        // }

        switch (m_controlMode) {
            case kHome:
                // Ignore soft limits, so we can go down till we hit the limit swithc
                m_homeVoltageRequst.IgnoreHardwareLimits = true;
                
                // Force the demand to always be negative
                m_homeVoltageRequst.Output = -Math.abs(m_demand);

                m_motor.setControl(m_homeVoltageRequst);
                break;
            case kOpenLoop:
                // Do openloop stuff here
                m_motor.setVoltage(m_demand);    
                break;
            case kMotionMagic:
                m_motionMagicRequest.Position = m_demand/MechanismConstants.kRotationToInches;
                m_motor.setControl(m_motionMagicRequest);
                break;
            case kStop:
                // Fall through to default
            default:
                m_motor.stopMotor();
        }      
    }
}
