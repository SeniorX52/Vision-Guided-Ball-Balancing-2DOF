package com.android.ball_balancing_system;
import com.google.gson.Gson;
import java.io.*;
import java.net.*;

public class JavaSocketClient {

    private Socket socket;
    private PrintWriter out;
    private final String host = "127.0.0.1";  // Python server address
    private final int port = 65432;           // Port number for the Python server
    private boolean isConnected=false;
    public JavaSocketClient() {

    }

    public void connect() {
        try {
            socket = new Socket(host, port);
            out = new PrintWriter(socket.getOutputStream(), true);
            System.out.println("Connected to Python server");
            isConnected=true;

        } catch (IOException e) {
            //e.printStackTrace();
            System.out.println("Cannot connect");
        }

    }

    public void disconnect() {
        if (out != null) out.close();
        if (socket != null) {
            try {
                socket.close();
                isConnected=false;
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }

    }
    public boolean isConnected(){
        return isConnected;
    }

    public void sendData(double desiredX, double desiredY,
                         double currentX, double currentY,
                         double current_velocity_X, double current_velocity_Y){

        Data data = new Data(desiredX, desiredY, currentX, currentY,
                current_velocity_X, current_velocity_Y);
        Gson gson = new Gson();
        String jsonData = gson.toJson(data);
        out.println(jsonData);
        //System.out.println("Data sent to Python: " + jsonData);
    }



    private static class ServoData {
        private final double servoX, servoY;

        public ServoData(double servoX, double servoY) {
            this.servoX = servoX;
            this.servoY = servoY;
        }
    }
    public static class SharedServoValues {
        private volatile double[] values = new double[]{0, 0};

        public synchronized void update(double[] newValues) {
            this.values = newValues;
        }

        public synchronized double[] get() {
            return values;
        }
    }
    public void startListeningInBackground(SharedServoValues sharedValues) {
        Thread listenerThread = new Thread(() -> {
            try {
                InputStream inputStream = socket.getInputStream();
                BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
                Gson gson = new Gson();
                StringBuilder dataBuilder = new StringBuilder();

                while (isConnected) {
                    if (reader.ready()) {
                        int character = reader.read();
                        if (character == -1) continue;
                        dataBuilder.append((char) character);

                        if (character == '\n') {
                            String incomingData = dataBuilder.toString().trim();
                            //System.out.println("Background received: " + incomingData);
                            dataBuilder.setLength(0); // reset buffer

                            try {
                                ServoData data = gson.fromJson(incomingData, ServoData.class);
                                sharedValues.update(new double[]{data.servoX, data.servoY});
                            } catch (Exception ex) {
                                System.err.println("Failed to parse incoming data: " + ex.getMessage());
                            }
                        }
                    } else {
                        Thread.sleep(10); // avoid busy loop
                    }
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        });
        listenerThread.setDaemon(true);
        listenerThread.start();
    }

    // Data class
    private static class Data {
        private final double desiredX, desiredY;
        private final double currentX, currentY;
        private final double current_velocity_X, current_velocity_Y;

        public Data(double desiredX, double desiredY, double currentX, double currentY,
                    double current_velocity_X, double current_velocity_Y) {
            this.desiredX = desiredX;
            this.desiredY = desiredY;
            this.currentX = currentX;
            this.currentY = currentY;
            this.current_velocity_X = current_velocity_X;
            this.current_velocity_Y = current_velocity_Y;
        }
    }
}