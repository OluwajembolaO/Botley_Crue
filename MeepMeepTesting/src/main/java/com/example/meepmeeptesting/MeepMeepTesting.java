package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.IOException;

public class MeepMeepTesting {
    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(800);

        // ========== CHOOSE YOUR PATH HERE ==========
        int selectedPath = 3; // Change to 1, 2, or 3
        // ===========================================

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // Define starting pose
        Pose2d startPose = new Pose2d(56, -8, Math.toRadians(45+180));

        switch (selectedPath) {
            case 1:
                // Path 1: Square Pattern
                myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                        .splineTo(new Vector2d(35, -50), Math.toRadians(-100))
                        .build());
                break;

            case 2:
                // Path 2: Zigzag Pattern
                myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                        .splineTo(new Vector2d(12, 50), Math.PI / 2)
                        .build());
                break;

            case 3:
                // Path 3: Figure 8 Pattern
                myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                        .splineTo(new Vector2d(-12, 50), Math.PI / 2)
                        .build());
                break;

            default:
                System.out.println("Invalid path selection! Using Path 1.");
                myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                        .lineToX(30)
                        .turn(Math.toRadians(90))
                        .lineToY(30)
                        .turn(Math.toRadians(90))
                        .lineToX(0)
                        .turn(Math.toRadians(90))
                        .lineToY(0)
                        .turn(Math.toRadians(90))
                        .build());
                break;
        }

        // --- Load and rotate the background manually ---
        BufferedImage field = ImageIO.read(MeepMeepTesting.class.getResourceAsStream("/background/season-2025-decode/field-2025-juice-black.png"));

        // Apply rotated background
        meepMeep.setBackground(field)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}