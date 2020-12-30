package as7262;

public interface IIOCompleteNotification {
    void setSensorData(AS7262Protocol.ColorData data, double sensor_timestamp);
}
