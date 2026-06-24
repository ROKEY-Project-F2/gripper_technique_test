# m0609_ros_bridge.py

from __future__ import annotations

from typing import Optional, Tuple

import omni.graph.core as og
import omni.usd

from isaacsim.core.utils.extensions import enable_extension


GRAPH_PATH = "/World/ROS_M0609_Graph"

COMMAND_TOPIC = "/m0609/move_command"
RESULT_TOPIC = "/m0609/move_result"

PICK_COMMAND_TOPIC = "/m0609/pick_command"

HAND_RAW_TOPIC = "/hand_raw"
HAND_TARGET_TOPIC = "/hand_xyz"
HAND_MODE_TOPIC = "/hand_mode"

STRING_TYPE = "token"


HAND_POSITION_QOS = r"""
{
    "history": "keepLast",
    "depth": 1,
    "reliability": "bestEffort",
    "durability": "volatile",
    "deadline": 0.0,
    "lifespan": 0.0,
    "liveliness": "systemDefault",
    "leaseDuration": 0.0
}
"""

HAND_MODE_QOS = r"""
{
    "history": "keepLast",
    "depth": 10,
    "reliability": "reliable",
    "durability": "volatile",
    "deadline": 0.0,
    "lifespan": 0.0,
    "liveliness": "systemDefault",
    "leaseDuration": 0.0
}
"""

_STATE_MACHINE = None

_LATEST_HAND_RAW: Optional[
    Tuple[float, float, float]
] = None
_HAND_RAW_SEQUENCE = 0

_LATEST_HAND_TARGET: Optional[
    Tuple[float, float, float]
] = None
_HAND_TARGET_SEQUENCE = 0

_LATEST_HAND_MODE = "TRACKING"
_HAND_MODE_SEQUENCE = 0


def get_state_machine():
    return _STATE_MACHINE


def set_latest_hand_raw(
    x: float,
    y: float,
    z: float,
) -> None:
    global _LATEST_HAND_RAW
    global _HAND_RAW_SEQUENCE

    _LATEST_HAND_RAW = (
        float(x),
        float(y),
        float(z),
    )
    _HAND_RAW_SEQUENCE += 1


def get_latest_hand_raw():
    return (
        _LATEST_HAND_RAW,
        _HAND_RAW_SEQUENCE,
    )


def set_latest_hand_target(
    x: float,
    y: float,
    z: float,
) -> None:
    global _LATEST_HAND_TARGET
    global _HAND_TARGET_SEQUENCE

    _LATEST_HAND_TARGET = (
        float(x),
        float(y),
        float(z),
    )
    _HAND_TARGET_SEQUENCE += 1


def get_latest_hand_target():
    return (
        _LATEST_HAND_TARGET,
        _HAND_TARGET_SEQUENCE,
    )


def set_latest_hand_mode(
    mode: str,
) -> None:
    global _LATEST_HAND_MODE
    global _HAND_MODE_SEQUENCE

    _LATEST_HAND_MODE = str(mode)
    _HAND_MODE_SEQUENCE += 1


def get_latest_hand_mode():
    return (
        _LATEST_HAND_MODE,
        _HAND_MODE_SEQUENCE,
    )


_COMMAND_SCRIPT = r"""
import json

from m0609_ros_bridge import get_state_machine


def compute(db):
    request_id = ""

    try:
        command = json.loads(str(db.inputs.command))

        request_id = str(
            command.get("request_id", "")
        )

        x = float(command["x"])
        y = float(command["y"])
        z = float(command["z"])

        state_machine = get_state_machine()

        if state_machine is None:
            raise RuntimeError(
                "State machine is not registered"
            )

        accepted, message = (
            state_machine.request_move(
                x,
                y,
                z,
            )
        )

        result = {
            "request_id": request_id,
            "accepted": bool(accepted),
            "message": str(message),
        }

    except Exception as error:
        result = {
            "request_id": request_id,
            "accepted": False,
            "message": str(error),
        }

    db.outputs.result = json.dumps(
        result,
        ensure_ascii=False,
    )

    return True
"""


_PICK_COMMAND_SCRIPT = r"""
from m0609_ros_bridge import get_state_machine


def compute(db):
    try:
        tray_index = int(db.inputs.command)

        state_machine = get_state_machine()

        if state_machine is None:
            raise RuntimeError(
                "State machine is not registered"
            )

        accepted, message = (
            state_machine.request_pick_command(
                tray_index
            )
        )

        print(
            "[ROS2 Bridge] pick command:"
            f" tray={tray_index},"
            f" accepted={accepted},"
            f" message={message}",
            flush=True,
        )

    except Exception as error:
        print(
            "[ROS2 Bridge] pick command error: "
            f"{error}",
            flush=True,
        )

    return True
"""


_HAND_RAW_SCRIPT = r"""
from m0609_ros_bridge import set_latest_hand_raw


def compute(db):
    try:
        set_latest_hand_raw(
            float(db.inputs.x),
            float(db.inputs.y),
            float(db.inputs.z),
        )
    except Exception as error:
        print(
            f"[ROS2 Bridge] hand raw error: {error}",
            flush=True,
        )

    return True
"""


_HAND_TARGET_SCRIPT = r"""
from m0609_ros_bridge import set_latest_hand_target


def compute(db):
    try:
        set_latest_hand_target(
            float(db.inputs.x),
            float(db.inputs.y),
            float(db.inputs.z),
        )
    except Exception as error:
        print(
            f"[ROS2 Bridge] hand target error: {error}",
            flush=True,
        )

    return True
"""


_HAND_MODE_SCRIPT = r"""
from m0609_ros_bridge import set_latest_hand_mode


def compute(db):
    try:
        set_latest_hand_mode(
            str(db.inputs.mode)
        )
    except Exception as error:
        print(
            f"[ROS2 Bridge] hand mode error: {error}",
            flush=True,
        )

    return True
"""


def _remove_existing_graph(
    simulation_app,
) -> None:
    stage = (
        omni.usd.get_context().get_stage()
    )

    graph_prim = stage.GetPrimAtPath(
        GRAPH_PATH
    )

    if not graph_prim.IsValid():
        return

    stage.RemovePrim(
        GRAPH_PATH
    )

    for _ in range(3):
        simulation_app.update()


def _create_script_ports() -> None:
    command_node = og.Controller.node(
        f"{GRAPH_PATH}/HandleCommand"
    )

    og.Controller.create_attribute(
        node=command_node,
        attr_name="inputs:command",
        attr_type=STRING_TYPE,
    )

    og.Controller.create_attribute(
        node=command_node,
        attr_name="outputs:result",
        attr_type=STRING_TYPE,
        attr_port=(
            og.AttributePortType
            .ATTRIBUTE_PORT_TYPE_OUTPUT
        ),
    )

    pick_node = og.Controller.node(
        f"{GRAPH_PATH}/HandlePickCommand"
    )

    og.Controller.create_attribute(
        node=pick_node,
        attr_name="inputs:command",
        attr_type="int",
    )

    for node_name in (
        "HandleHandRaw",
        "HandleHandTarget",
    ):
        node = og.Controller.node(
            f"{GRAPH_PATH}/{node_name}"
        )

        for axis in ("x", "y", "z"):
            og.Controller.create_attribute(
                node=node,
                attr_name=f"inputs:{axis}",
                attr_type="double",
            )

    mode_node = og.Controller.node(
        f"{GRAPH_PATH}/HandleHandMode"
    )

    og.Controller.create_attribute(
        node=mode_node,
        attr_name="inputs:mode",
        attr_type=STRING_TYPE,
    )


def setup_m0609_ros_bridge(
    state_machine,
    simulation_app,
) -> None:
    global _STATE_MACHINE
    _STATE_MACHINE = state_machine

    enable_extension(
        "isaacsim.ros2.bridge"
    )
    enable_extension(
        "omni.graph.scriptnode"
    )

    for _ in range(3):
        simulation_app.update()

    _remove_existing_graph(
        simulation_app
    )

    graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": GRAPH_PATH,
            "evaluator_name": "execution",
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                (
                    "OnPlaybackTick",
                    "omni.graph.action.OnPlaybackTick",
                ),
                (
                    "ROS2Context",
                    "isaacsim.ros2.bridge.ROS2Context",
                ),

                (
                    "CommandSubscriber",
                    "isaacsim.ros2.bridge.ROS2Subscriber",
                ),
                (
                    "HandleCommand",
                    "omni.graph.scriptnode.ScriptNode",
                ),
                (
                    "ResultPublisher",
                    "isaacsim.ros2.bridge.ROS2Publisher",
                ),

                (
                    "PickCommandSubscriber",
                    "isaacsim.ros2.bridge.ROS2Subscriber",
                ),
                (
                    "HandlePickCommand",
                    "omni.graph.scriptnode.ScriptNode",
                ),

                (
                    "HandRawSubscriber",
                    "isaacsim.ros2.bridge.ROS2Subscriber",
                ),
                (
                    "HandleHandRaw",
                    "omni.graph.scriptnode.ScriptNode",
                ),

                (
                    "HandTargetSubscriber",
                    "isaacsim.ros2.bridge.ROS2Subscriber",
                ),
                (
                    "HandleHandTarget",
                    "omni.graph.scriptnode.ScriptNode",
                ),

                (
                    "HandModeSubscriber",
                    "isaacsim.ros2.bridge.ROS2Subscriber",
                ),
                (
                    "HandleHandMode",
                    "omni.graph.scriptnode.ScriptNode",
                ),
            ],

            og.Controller.Keys.SET_VALUES: [
                (
                    "CommandSubscriber.inputs:messagePackage",
                    "std_msgs",
                ),
                (
                    "CommandSubscriber.inputs:messageSubfolder",
                    "msg",
                ),
                (
                    "CommandSubscriber.inputs:messageName",
                    "String",
                ),
                (
                    "CommandSubscriber.inputs:topicName",
                    COMMAND_TOPIC,
                ),

                (
                    "ResultPublisher.inputs:messagePackage",
                    "std_msgs",
                ),
                (
                    "ResultPublisher.inputs:messageSubfolder",
                    "msg",
                ),
                (
                    "ResultPublisher.inputs:messageName",
                    "String",
                ),
                (
                    "ResultPublisher.inputs:topicName",
                    RESULT_TOPIC,
                ),
                (
                    "HandleCommand.inputs:script",
                    _COMMAND_SCRIPT,
                ),

                (
                    "PickCommandSubscriber.inputs:messagePackage",
                    "std_msgs",
                ),
                (
                    "PickCommandSubscriber.inputs:messageSubfolder",
                    "msg",
                ),
                (
                    "PickCommandSubscriber.inputs:messageName",
                    "Int32",
                ),
                (
                    "PickCommandSubscriber.inputs:topicName",
                    PICK_COMMAND_TOPIC,
                ),
                (
                    "HandlePickCommand.inputs:script",
                    _PICK_COMMAND_SCRIPT,
                ),

                (
                    "HandRawSubscriber.inputs:messagePackage",
                    "geometry_msgs",
                ),
                (
                    "HandRawSubscriber.inputs:messageSubfolder",
                    "msg",
                ),
                (
                    "HandRawSubscriber.inputs:messageName",
                    "Point",
                ),
                (
                    "HandRawSubscriber.inputs:topicName",
                    HAND_RAW_TOPIC,
                ),
                (
                    "HandRawSubscriber.inputs:qosProfile",
                    HAND_POSITION_QOS,
                ),
                (
                    "HandleHandRaw.inputs:script",
                    _HAND_RAW_SCRIPT,
                ),

                (
                    "HandTargetSubscriber.inputs:messagePackage",
                    "geometry_msgs",
                ),
                (
                    "HandTargetSubscriber.inputs:messageSubfolder",
                    "msg",
                ),
                (
                    "HandTargetSubscriber.inputs:messageName",
                    "Point",
                ),
                (
                    "HandTargetSubscriber.inputs:topicName",
                    HAND_TARGET_TOPIC,
                ),
                (
                    "HandTargetSubscriber.inputs:qosProfile",
                    HAND_POSITION_QOS,
                ),
                (
                    "HandleHandTarget.inputs:script",
                    _HAND_TARGET_SCRIPT,
                ),

                (
                    "HandModeSubscriber.inputs:messagePackage",
                    "std_msgs",
                ),
                (
                    "HandModeSubscriber.inputs:messageSubfolder",
                    "msg",
                ),
                (
                    "HandModeSubscriber.inputs:messageName",
                    "String",
                ),
                (
                    "HandModeSubscriber.inputs:topicName",
                    HAND_MODE_TOPIC,
                ),
                (
                    "HandModeSubscriber.inputs:qosProfile",
                    HAND_MODE_QOS,
                ),
                (
                    "HandleHandMode.inputs:script",
                    _HAND_MODE_SCRIPT,
                ),
            ],

            og.Controller.Keys.CONNECT: [
                (
                    "OnPlaybackTick.outputs:tick",
                    "CommandSubscriber.inputs:execIn",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "PickCommandSubscriber.inputs:execIn",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "HandRawSubscriber.inputs:execIn",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "HandTargetSubscriber.inputs:execIn",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "HandModeSubscriber.inputs:execIn",
                ),

                (
                    "ROS2Context.outputs:context",
                    "CommandSubscriber.inputs:context",
                ),
                (
                    "ROS2Context.outputs:context",
                    "ResultPublisher.inputs:context",
                ),
                (
                    "ROS2Context.outputs:context",
                    "PickCommandSubscriber.inputs:context",
                ),
                (
                    "ROS2Context.outputs:context",
                    "HandRawSubscriber.inputs:context",
                ),
                (
                    "ROS2Context.outputs:context",
                    "HandTargetSubscriber.inputs:context",
                ),
                (
                    "ROS2Context.outputs:context",
                    "HandModeSubscriber.inputs:context",
                ),
            ],
        },
    )

    for _ in range(10):
        simulation_app.update()

    _create_script_ports()

    for _ in range(3):
        simulation_app.update()

    def path(
        node: str,
        port: str,
    ) -> str:
        return (
            f"{GRAPH_PATH}/{node}.{port}"
        )

    og.Controller.edit(
        graph,
        {
            og.Controller.Keys.CONNECT: [
                (
                    path(
                        "CommandSubscriber",
                        "outputs:execOut",
                    ),
                    path(
                        "HandleCommand",
                        "inputs:execIn",
                    ),
                ),
                (
                    path(
                        "CommandSubscriber",
                        "outputs:data",
                    ),
                    path(
                        "HandleCommand",
                        "inputs:command",
                    ),
                ),
                (
                    path(
                        "HandleCommand",
                        "outputs:execOut",
                    ),
                    path(
                        "ResultPublisher",
                        "inputs:execIn",
                    ),
                ),
                (
                    path(
                        "HandleCommand",
                        "outputs:result",
                    ),
                    path(
                        "ResultPublisher",
                        "inputs:data",
                    ),
                ),

                (
                    path(
                        "PickCommandSubscriber",
                        "outputs:execOut",
                    ),
                    path(
                        "HandlePickCommand",
                        "inputs:execIn",
                    ),
                ),
                (
                    path(
                        "PickCommandSubscriber",
                        "outputs:data",
                    ),
                    path(
                        "HandlePickCommand",
                        "inputs:command",
                    ),
                ),

                (
                    path(
                        "HandRawSubscriber",
                        "outputs:execOut",
                    ),
                    path(
                        "HandleHandRaw",
                        "inputs:execIn",
                    ),
                ),
                (
                    path(
                        "HandRawSubscriber",
                        "outputs:x",
                    ),
                    path(
                        "HandleHandRaw",
                        "inputs:x",
                    ),
                ),
                (
                    path(
                        "HandRawSubscriber",
                        "outputs:y",
                    ),
                    path(
                        "HandleHandRaw",
                        "inputs:y",
                    ),
                ),
                (
                    path(
                        "HandRawSubscriber",
                        "outputs:z",
                    ),
                    path(
                        "HandleHandRaw",
                        "inputs:z",
                    ),
                ),

                (
                    path(
                        "HandTargetSubscriber",
                        "outputs:execOut",
                    ),
                    path(
                        "HandleHandTarget",
                        "inputs:execIn",
                    ),
                ),
                (
                    path(
                        "HandTargetSubscriber",
                        "outputs:x",
                    ),
                    path(
                        "HandleHandTarget",
                        "inputs:x",
                    ),
                ),
                (
                    path(
                        "HandTargetSubscriber",
                        "outputs:y",
                    ),
                    path(
                        "HandleHandTarget",
                        "inputs:y",
                    ),
                ),
                (
                    path(
                        "HandTargetSubscriber",
                        "outputs:z",
                    ),
                    path(
                        "HandleHandTarget",
                        "inputs:z",
                    ),
                ),

                (
                    path(
                        "HandModeSubscriber",
                        "outputs:execOut",
                    ),
                    path(
                        "HandleHandMode",
                        "inputs:execIn",
                    ),
                ),
                (
                    path(
                        "HandModeSubscriber",
                        "outputs:data",
                    ),
                    path(
                        "HandleHandMode",
                        "inputs:mode",
                    ),
                ),
            ],
        },
    )

    for _ in range(3):
        simulation_app.update()

    print(
        "[ROS2 Bridge] ready:\n"
        f"  pick command: {PICK_COMMAND_TOPIC} "
        "(std_msgs/msg/Int32, send 7)\n"
        f"  hand raw: {HAND_RAW_TOPIC}\n"
        f"  hand target: {HAND_TARGET_TOPIC}\n"
        f"  hand mode: {HAND_MODE_TOPIC} "
        "(HOME -> PLACE)",
        flush=True,
    )