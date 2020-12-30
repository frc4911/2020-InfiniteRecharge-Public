package vl53l0x;

public interface IIOCompleteNotification {
    void setLidarData(VL53L0XProtocol.LidarData data, double sensor_timestamp);
}
