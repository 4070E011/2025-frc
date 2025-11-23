package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class Constanats {
    public static final double kDefaultVoltage = 12.0;

    public static class RobotConstants {
        public static final double kMaxSpeed = 4.5; // m/s
        public static final double kMaxAngularSpeed = Math.PI * 2;
    }

    public static class DriverConstanats {
        public static final double kDriveSpeed = 0.5;   
        public static final double kTurnSpeed = 0.5;    
    }

    public static class CANDevicesID {
        public static final int kLFTurnMotorID = 1;
        public static final int kRFTurnMotorID = 2;
        public static final int kRRTurnMotorID = 3;
        public static final int kLRTurnMotorID = 4;
        public static final int kLFDriveMotorID = 5;
        public static final int kRFDriveMotorID = 6;
        public static final int kRRDriveMotorID = 7;
        public static final int kLRDriveMotorID = 8;
        public static final int kLFCancoderID = 9;
        public static final int kRFCancoderID = 10;
        public static final int kRRCancoderID = 11;
        public static final int kLRCancoderID = 12;
    }

    public static class Offsets {
        public static final double kLFCANCoderOffsets = 0.279296875;
        public static final double kRFCANCoderOffsets = -0.046142578125;
        public static final double kRRCANCoderOffsets = 0.271728515625;
        public static final double kLRCANCoderOffsets = 0.44921875;
    }

    public static class Configurations {
        public static TalonFXConfiguration getDriveMotorConfig(boolean inverted) {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            configs.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
            configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            return configs;
        }

        public static TalonFXConfiguration getTurnMotorConfig(int CANcoderID, boolean inverted) {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            configs.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
            configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            return configs;
        }

        public static CANcoderConfiguration getTurnEncoderConfig(double offset, boolean inverted) {
            CANcoderConfiguration configs = new CANcoderConfiguration();
            configs.MagnetSensor.SensorDirection = inverted ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;
            configs.MagnetSensor.MagnetOffset = offset;

            return configs;
        }
        
    }
}