module com.android.ball_balancing_system {
    requires javafx.controls;
    requires javafx.fxml;
    requires javafx.web;
    requires org.controlsfx.controls;
    requires com.dlsc.formsfx;
    requires net.synedra.validatorfx;
    requires org.kordamp.ikonli.javafx;
    requires org.kordamp.bootstrapfx.core;
    requires eu.hansolo.tilesfx;
    requires com.almasb.fxgl.all;
    requires com.fazecast.jSerialComm;
    requires com.google.gson;
    requires commons.math3;
    requires org.apache.commons.geometry.euclidean;
    requires ejml.simple;
    requires org.locationtech.jts;

    opens com.android.ball_balancing_system to javafx.fxml, com.google.gson;
    exports com.android.ball_balancing_system;
}