package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.IOException;

public class MeepMeepTesting {
    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .build());

        // --- Load and rotate the background manually ---
        BufferedImage field = ImageIO.read(MeepMeepTesting.class.getResourceAsStream(
                "/background/season-2025-decode/field-2025-juice-black.png"
        ));

        // Rotate by 90 degrees (change to 180 or -90 if needed)
        BufferedImage rotatedField = rotateImage(field, 90);

        // Apply rotated background
        meepMeep.setBackground(rotatedField)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    /**
     * Rotates an image by a given angle (in degrees).
     */
    private static BufferedImage rotateImage(BufferedImage img, double degrees) {
        double radians = Math.toRadians(degrees);
        double sin = Math.abs(Math.sin(radians));
        double cos = Math.abs(Math.cos(radians));
        int newWidth = (int) Math.floor(img.getWidth() * cos + img.getHeight() * sin);
        int newHeight = (int) Math.floor(img.getHeight() * cos + img.getWidth() * sin);

        BufferedImage rotated = new BufferedImage(newWidth, newHeight, img.getType());
        Graphics2D g2d = rotated.createGraphics();
        g2d.translate(newWidth / 2.0, newHeight / 2.0);
        g2d.rotate(radians);
        g2d.translate(-img.getWidth() / 2.0, -img.getHeight() / 2.0);
        g2d.drawImage(img, 0, 0, null);
        g2d.dispose();
        return rotated;
    }
}
