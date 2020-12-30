package vl53l0x;

interface IIOProvider {
    public boolean  isConnected();
    public void     run();
    public void     stop();
    public double getUpdateCount();
    public void	    enableLogging(boolean enable);
    public boolean  isInitialized();
}
