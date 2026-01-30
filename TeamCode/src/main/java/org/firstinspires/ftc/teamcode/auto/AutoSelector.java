package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoSelector {

    boolean lastA, lastX;

    public void update(Gamepad gp, Telemetry tel, AutoConfig cfg) {

        if (gp.a && !lastA) {
            cfg.alliance =
                    (cfg.alliance == AutoConfig.Alliance.RED)
                            ? AutoConfig.Alliance.BLUE
                            : AutoConfig.Alliance.RED;
        }

        if (gp.x && !lastX) {
            cfg.route = AutoConfig.Route.values()
                    [(cfg.route.ordinal() + 1)
                    % AutoConfig.Route.values().length];
        }

        lastA = gp.a;
        lastX = gp.x;

        tel.addLine("=== AUTO SELECT ===");
        tel.addData("Alliance", cfg.alliance);
        tel.addData("Route", cfg.route);
        tel.addLine("A = Alliance | X = Route");
        tel.update();
    }
}
