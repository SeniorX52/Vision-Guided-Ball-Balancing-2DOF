package com.android.ball_balancing_system;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.stage.Stage;

import java.io.IOException;

public class HelloApplication extends Application {
    static {
        System.setProperty("prism.order", "d3d,sw");
        System.setProperty("prism.vsync", "true");
        System.setProperty("prism.forceGPU", "true");
    }

    @Override
    public void start(Stage stage) throws IOException {
        System.out.println("Prism Order: " + System.getProperty("prism.order"));
        System.out.println("Prism ForceGPU: " + System.getProperty("prism.forceGPU"));
        System.out.println("Prism pipeline: " + System.getProperty("prism.verbose"));
        System.out.println("Graphics device: " + javafx.stage.Screen.getPrimary().getDpi());
        System.out.println("D3D Adapter: " + System.getProperty("prism.d3d.adapter"));
        stage.setTitle("Ball Balancing System");
        SceneManager sceneManager=new SceneManager(stage);
        SceneManager.switchToMain();
    }

    public static void main(String[] args) {
        launch();
    }
}