package as7262;

public class AS7262Protocol {
    public static class ColorData {
        public static int[] rawValues = new int[AS7262Registers.AS726x_NUM_CHANNELS + 1];
        public static float[] calibratedValues = new float [AS7262Registers.AS726x_NUM_CHANNELS + 1];
        public int temp_c;
    }
}
