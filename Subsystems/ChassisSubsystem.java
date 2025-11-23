package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChassisSubsystem extends SubsystemBase {
    // 定義四顆馬達控制器 (SparkMax)
    private SparkMax left_motor_1;
    private SparkMax left_motor_2;
    private SparkMax right_motor_1;
    private SparkMax right_motor_2;
    
    public ChassisSubsystem() {
        // 初始化馬達 (ID, 類型: 有刷馬達)
        left_motor_1 = new SparkMax(3, MotorType.kBrushed); 
        left_motor_2 = new SparkMax(4, MotorType.kBrushed);
        right_motor_1 = new SparkMax(1, MotorType.kBrushed);
        right_motor_2 = new SparkMax(2, MotorType.kBrushed);
    }

    // 傳統 Arcade Drive (前後 + 左右)
    public void drive(double x, double y) {
        // 計算左右輪動力
        // x: 前後, y: 轉向
        double l_power = bounding(x + y, 1, -1);
        double r_power = bounding(x - y, 1, -1);

        // 設定馬達輸出 (左側通常需要反轉，這裡視實際接線而定)
        left_motor_1.set(l_power*-1.0);
        left_motor_2.set(l_power*-1.0);
        right_motor_1.set(r_power);
        right_motor_2.set(r_power);
    }

    // 數值限制函式 (Clamp)：確保輸出不會超過馬達極限 (+1 ~ -1)
    private double bounding(double value, double max, double min) {
        if (value > max) return max;
        else if(value < min) return min;
        else return value;
    }
}