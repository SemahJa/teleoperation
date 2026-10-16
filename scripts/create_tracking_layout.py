#!/usr/bin/env python3
"""Create a PlotJuggler 3.x live dashboard for one or more arm namespaces."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

COLORS = ("#0072b2", "#e69f00", "#009e73", "#cc79a7")


def layout(namespaces):
    root = ET.Element("root")
    tabs = ET.SubElement(
        root, "tabbed_widget", name="Main Window", parent="main_window"
    )

    def page(name, rows):
        tab = ET.SubElement(tabs, "Tab", tab_name=name, containers="1")
        container = ET.SubElement(tab, "Container")
        outer = ET.SubElement(
            container,
            "DockSplitter",
            orientation="-",
            count=str(len(rows)),
            sizes=";".join(["1"] * len(rows)),
        )
        for row in rows:
            split = ET.SubElement(
                outer,
                "DockSplitter",
                orientation="|",
                count=str(len(row)),
                sizes=";".join(["1"] * len(row)),
            )
            for title, series in row:
                area = ET.SubElement(split, "DockArea", name=title)
                plot = ET.SubElement(
                    area,
                    "plot",
                    style="Lines",
                    mode="TimeSeries",
                    flip_x="false",
                    flip_y="false",
                )
                ET.SubElement(plot, "range", left="0", right="10", bottom="-1", top="1")
                ET.SubElement(plot, "limitY")
                for i, series_name in enumerate(series):
                    ET.SubElement(
                        plot, "curve", name=series_name, color=COLORS[i % len(COLORS)]
                    )

    for namespace in namespaces:
        ns = "/" + namespace.strip("/") if namespace.strip("/") else ""
        prefix = ns + "/tracking_status/"
        name = ns or "/"
        curves = lambda suffixes: [prefix + suffix for suffix in suffixes]
        pose = [
            (
                f"{axis.upper()} position [m]: request / reference / measured",
                curves(
                    [
                        f"{kind}_pose/position/{axis}"
                        for kind in ("requested", "reference", "measured")
                    ]
                ),
            )
            for axis in "xyz"
        ]
        errors = [
            ("Position error [m]", curves(["position_error_norm"])),
            ("Orientation error [rad]", curves(["rotation_error_angle"])),
            (
                "Cartesian active / home / request twist valid",
                curves(["motion_enabled", "homing_active", "requested_twist_valid"]),
            ),
        ]
        page(name + " Pose", [pose, errors])
        rows = []
        for part, unit in [("linear", "m/s"), ("angular", "rad/s")]:
            rows.append(
                [
                    (
                        f"{part} {axis} [{unit}]: request / reference / measured",
                        curves(
                            [
                                f"{kind}_twist/{part}/{axis}"
                                for kind in ("requested", "reference", "measured")
                            ]
                        ),
                    )
                    for axis in "xyz"
                ]
            )
        page(name + " Velocity", rows)
        joints = [
            (
                f"J{i+1} torque [Nm]; measured includes gravity",
                curves(
                    [
                        f"{kind}_torque.{i}"
                        for kind in (
                            "nominal",
                            "commanded",
                            "robot_desired",
                            "measured",
                        )
                    ]
                ),
            )
            for i in range(7)
        ]
        joints.append(
            (
                "Hardware torque valid / previous command available",
                curves(["robot_torque_valid", "previous_command_valid"]),
            )
        )
        page(name + " Four torques", [joints[:4], joints[4:]])
        page(
            name + " Diagnostics",
            [
                [
                    ("Command age [ms]", curves(["command_age_ms"])),
                    (
                        "Normalized safety intervention",
                        curves(["normalized_torque_intervention"]),
                    ),
                    (
                        "Torque rate / magnitude / fallback",
                        curves(
                            [
                                "torque_rate_limited",
                                "torque_magnitude_limited",
                                "safety_fallback",
                            ]
                        ),
                    ),
                ],
                [
                    (
                        "Monitoring queue drops (cumulative)",
                        curves(["monitor_queue_drops"]),
                    ),
                    (
                        "Linear velocity error [m/s]",
                        curves(["linear_velocity_error_norm"]),
                    ),
                    (
                        "Angular velocity error [rad/s]",
                        curves(["angular_velocity_error_norm"]),
                    ),
                ],
            ],
        )
    ET.SubElement(tabs, "currentTabIndex", index="0")
    ET.SubElement(root, "use_relative_time_offset", enabled="1")
    ET.SubElement(root, "streaming_buffer_size", value="30")
    ET.SubElement(root, "Plugins")
    ET.SubElement(root, "customMathEquations")
    ET.SubElement(root, "snippets")
    return ET.ElementTree(root)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", action="append", default=[])
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    tree = layout(args.namespace or ["/panda_1"])
    ET.indent(tree, space="  ")
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
