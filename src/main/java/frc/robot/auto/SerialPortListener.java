// package frc.robot.auto;

// import edu.wpi.first.wpilibj.SerialPort;
// import edu.wpi.first.wpilibj.SerialPort.Port;
// import java.nio.charset.StandardCharsets;


// public class SerialPortListener {

//     private static final int BAUD_RATE = 115200;
//     private static final Port PORT = Port.kUSB;
//     public static int xValue = 0;
//     public static int yValue = 0;

//     private SerialPort serialPort;

//     public SerialPortListener() {
//         serialPort = new SerialPort(BAUD_RATE, PORT);
//     }

//     public void periodic() {
//         // Read data from the serial port
//         byte[] buffer = serialPort.read(64); // Read up to 64 bytes
//         if (buffer.length > 0) {
//             String data = new String(buffer, StandardCharsets.UTF_8);
//             int[] values = parseCoordinates(data);
//             int x = values[0];
//             int y = values[1];
//         }
//     }

//     private int[] parseCoordinates(String data) {
//         // Split the data by newline to get each line
//         String[] lines = data.split("\n");

//         for (String line : lines) {
//             // Split the line by comma to separate X and Y parts
//             String[] parts = line.split(", ");

//             // Check if the line contains both X and Y parts
//             if (parts.length == 2) {
//                 // Extract X and Y values
//                 String xPart = parts[0];
//                 String yPart = parts[1];

//                 // Extract the numeric values from X and Y parts
//                 int xValue = extractValue(xPart, "X: ");
//                 int yValue = extractValue(yPart, "Y: ");

//             }
//         }    
//     return new int[]{xValue, yValue};            

//     }

//     private static int extractValue(String part, String prefix) {
//         // Remove the prefix from the part
//         String valueString = part.replace(prefix, "");

//         // Convert the remaining string to an integer
//         int value = Integer.parseInt(valueString);

//         return value;
//     }
// }
