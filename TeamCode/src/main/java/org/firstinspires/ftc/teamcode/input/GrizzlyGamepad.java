package org.firstinspires.ftc.teamcode.input;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

public class GrizzlyGamepad {

    private final Gamepad gamepad;
    private final ElapsedTime timer;

    private final HashMap<Button, ButtonAction> buttonActions;

    public GrizzlyGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;

        timer = new ElapsedTime();
        buttonActions = new HashMap<>();
        timer.reset();

        for (Button button : Button.values()) {
            buttonActions.put(button, ButtonAction.NONE);
        }
    }

    public float getAxis(Axis axis) {
        String name = axis.toString().toLowerCase();
        try {
            return gamepad.getClass().getDeclaredField(name).getFloat(gamepad);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (NoSuchFieldException e) {
            e.printStackTrace();
        }

        return 0;
    }

    public boolean getButton(Button button) {
        String name = button.toString().toLowerCase();
        try {
            return gamepad.getClass().getDeclaredField(name).getBoolean(gamepad);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (NoSuchFieldException e) {
            e.printStackTrace();
        }

        return false;
    }

    public ButtonAction getButtonAction(Button button) {
        return buttonActions.get(button);
    }

    public void update() {
        for (Button button : Button.values()) {
            boolean pressed = getButton(button);
            ButtonAction action = buttonActions.get(button);

            if (pressed) {
                if (action == ButtonAction.NONE) {
                    buttonActions.put(button, ButtonAction.PRESS);
                } else {
                    buttonActions.put(button, ButtonAction.HELD);
                }
            } else {
                if (action == ButtonAction.PRESS || action == ButtonAction.HELD) {
                    buttonActions.put(button, ButtonAction.RELEASE);
                } else {
                    buttonActions.put(button, ButtonAction.NONE);
                }
            }
        }
    }

    public void setLed(double r, double g, double b) {
        gamepad.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS);
    }

    public void rumble(int durationMs) {
        gamepad.rumble(durationMs);
    }

    public void rumble(double rumble1, double rumble2, int durationMs) {
        gamepad.rumble(rumble1, rumble2, durationMs);
    }
}
