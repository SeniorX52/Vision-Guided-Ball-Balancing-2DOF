package com.android.ball_balancing_system;

import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;

import java.io.IOException;
import java.util.HashMap;
import java.util.Objects;

public class SceneManager {
    private static Stage primaryStage;
    private static Stage graphStage;
    public SceneManager(Stage primaryStage){
        SceneManager.primaryStage =primaryStage;
        scenes=new HashMap<>();
        graphStage=new Stage();
        graphStage.setTitle("Graphs");
    }
    private static HashMap<String,Scene> scenes;
    public static void switchToMain()  {
        try{
            if(scenes.containsKey("MainPage.fxml")){
                primaryStage.setScene(scenes.get("MainPage.fxml"));
                primaryStage.show();
            }
            else{
                FXMLLoader loader = new FXMLLoader(Objects.requireNonNull(SceneManager.class.getResource("MainPage.fxml")));
                Parent root = loader.load();
                Scene scene = new Scene(root);
                scenes.put("MainPage.fxml",scene);
                primaryStage.setScene(scene);
                primaryStage.show();
            }

        }
        catch (IOException e){
            e.printStackTrace();
        }
    }
    public static void switchToGraphs()  {
        try{
            if(scenes.containsKey("Graphs.fxml")){
                graphStage.setScene(scenes.get("Graphs.fxml"));
                graphStage.setAlwaysOnTop(true);
                graphStage.setResizable(true);
                graphStage.show();
            }
            else{
                FXMLLoader loader = new FXMLLoader(Objects.requireNonNull(SceneManager.class.getResource("Graphs.fxml")));
                Parent root = loader.load();
                Scene scene = new Scene(root);
                scenes.put("Graphs.fxml",scene);
                graphStage.setScene(scene);
                graphStage.setAlwaysOnTop(true);
                graphStage.setResizable(true);
                graphStage.show();
            }

        }
        catch (IOException e){
            e.printStackTrace();
        }
    }
}
