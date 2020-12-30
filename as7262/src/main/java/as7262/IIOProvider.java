package as7262;

interface IIOProvider {
    public boolean  isConnected();
    public void     run();
    public void     stop();
    public double   getUpdateCount();
    public void	    enableLogging(boolean enable);
    public boolean  isInitialized();
    // Sensor specific control

    public void setGain(byte gain);
    public void setIntegrationTime(byte time);
    public void enableDrvLed(boolean enable);
    public void setDrvCurrentLimit(byte limit);
    public void enableIndicateLED(boolean enable);
    public void setIndicateCurrentLimit(byte limit);
    public void setConversionType(byte type);
    public void reset();
}
