package frc.robot.Modules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constanats;
import frc.robot.Constanats.RobotConstants;

public class SwerveModule {
    // 定義硬體物件：驅動馬達、轉向馬達、絕對編碼器
    private TalonFX driveMotor;
    private TalonFX turnMotor;
    private CANcoder encoder;

    // PID 控制器：用來讓轉向馬達精準轉到目標角度
    private PIDController pid;
    private double invert;

    // 建構子：初始化硬體和參數
    public SwerveModule(int driveID, int turnID, int encoderID) {
        this.driveMotor = new TalonFX(driveID);
        this.turnMotor = new TalonFX(turnID);
        this.encoder = new CANcoder(encoderID);

        invert = 1.0;

        // 初始化 PID 參數 (Proportional, Integral, Derivative)
        // 目前只有 P=0.1，代表誤差越大修正力道越大
        pid = new PIDController(0.05, 0.0, 0.0);
        
        /*
         * PID 調整建議：
         * p: 0 ~ 1 (主要控制力道)
         * i: 1e-3 以下 (消除微小誤差，通常 Swerve 不需要)
         * d: 0 ~ 1 (抑制震盪)
         */

        // 啟用連續輸入：告訴 PID -180度和 180度是同一個點
        // 這樣輪子從 170度轉到 -170度只會轉 20度，不會繞一大圈
        pid.enableContinuousInput(-180, 180);
    }

    // --- 核心邏輯：路徑最佳化 ---
    // 判斷是「轉過頭」比較快，還是「倒車」比較快
    private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        // 1. 計算目標角度與當前角度的差值
        Rotation2d delta = desiredState.angle.minus(currentAngle);
        
        // 2. 如果誤差絕對值 > 90 度 (代表轉過頭了)
        // 使用 Math.abs(delta.getDegrees()) 有時會遇到邊界問題，建議用餘弦值判斷
        // 如果 cos(delta) < 0，代表夾角超過 90 度
        if (Math.abs(delta.getDegrees()) > 90.0) {
            // 直接回傳一個「速度相反、角度加 180」的新狀態
            // 注意：這裡不需要修改全域變數 invert，直接回傳負的速度即可
            return new SwerveModuleState(
                -desiredState.speedMetersPerSecond, // 速度變負
                desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)) // 角度轉 180
            );
        }
        else {
            // 如果誤差 < 90 度，直接回傳原本的狀態
            return new SwerveModuleState(
                desiredState.speedMetersPerSecond, 
                desiredState.angle);
        }
    }
    
    // 設定模組狀態 (主要被外部呼叫的函式)
    public void setState(SwerveModuleState desiredState) {
        // 1. 讀取絕對編碼器的當前角度
        Rotation2d currentAngle = Rotation2d.fromRotations(encoder.getPosition().getValueAsDouble());
        
        // 2. 呼叫上面的 optimize 函式計算最佳路徑
        SwerveModuleState optimizedState = optimize(desiredState, currentAngle);
        
        // 3. 計算 PID 輸出 (轉向馬達該給多少電壓)
        double turnOutput = pid.calculate(currentAngle.getDegrees(), optimizedState.angle.getDegrees());
        
        // 4. 計算驅動馬達輸出比例 (目前速度 / 最大速度)
        double drivePercent = optimizedState.speedMetersPerSecond / RobotConstants.kMaxSpeed;
    
        // 5. 設定馬達電壓 (TalonFX 接受電壓控制)
        driveMotor.setVoltage(drivePercent * Constanats.kDefaultVoltage); // 假設電池 12V
        turnMotor.setVoltage(turnOutput); 
    }

    public SwerveModuleState getState() {
        Rotation2d currentAngle = Rotation2d.fromRotations(encoder.getPosition().getValueAsDouble());
        return new SwerveModuleState( 
            (1.0 / 8.14) * 0.1 * Math.PI,
            currentAngle
        );
    }

    public SwerveModulePosition getPosition() {
        Rotation2d currentAngle = Rotation2d.fromRotations(encoder.getPosition().getValueAsDouble());
        return new  SwerveModulePosition( 
            driveMotor.getPosition().getValueAsDouble() / 8.14 * 0.1 * Math.PI,
            currentAngle
        );
    }

    public void setConfigs(TalonFXConfiguration driveMotorConfig, TalonFXConfiguration turnMotorConfig, CANcoderConfiguration CANcoderConfig) {
        driveMotor.getConfigurator().apply(driveMotorConfig);
        turnMotor.getConfigurator().apply(turnMotorConfig);
        encoder.getConfigurator().apply(CANcoderConfig);
    }
}