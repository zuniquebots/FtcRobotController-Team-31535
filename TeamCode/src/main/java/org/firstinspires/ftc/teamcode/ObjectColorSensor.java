package org.firstinspires.ftc.teamcode;

public class ObjectColorSensor{
        public static boolean isGreenOrPurple(int R, int G, int B) {
            boolean isGreen = G > R * 1.5 && G > B * 1.5 && G > 50; // Green detection
            boolean isPurple = R > G * 1.5 && B > G * 1.5 && R + B > 150; // Purple detection
            // True if both green and purple are detected;
            return isGreen || isPurple;
        }

        public static void main(String[] args) {
            // Example RGB values
            int R = 120, G = 200, B = 80; // Sample it's for green ball
            int R2 = 180, G2 = 50, B2 = 200; // Sample it's for purple ball

            // Check if both colors are detected
            boolean result = isGreenOrPurple(R, G, B) || isGreenOrPurple(R2, G2, B2);

            System.out.println("Green and Purple detected:" + result);
        }
}
