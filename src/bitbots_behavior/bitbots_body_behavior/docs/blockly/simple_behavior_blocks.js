const SIMPLE_BEHAVIOR_TOOLBOX = {
  kind: "categoryToolbox",
  contents: [
    {
      kind: "category",
      name: "Program",
      colour: "#4C97FF",
      contents: [
        {kind: "block", type: "sb_main"},
        {kind: "block", type: "sb_forever"},
        {kind: "block", type: "controls_repeat_ext"},
        {kind: "block", type: "time_sleep"},
      ],
    },
    {
      kind: "category",
      name: "Movement",
      colour: "#0FBD8C",
      contents: [
        {kind: "block", type: "sb_set_goal_pose"},
        {kind: "block", type: "sb_set_goal_pose_from_pose"},
        {kind: "block", type: "sb_stop"},
      ],
    },
    {
      kind: "category",
      name: "Ball actions",
      colour: "#E65C5C",
      contents: [
        {kind: "block", type: "sb_kick_ball"},
      ],
    },
    {
      kind: "category",
      name: "Head control",
      colour: "#8E5CB7",
      contents: [{kind: "block", type: "sb_set_head_mode"}],
    },
    {
      kind: "category",
      name: "Sensing",
      colour: "#2D9CDB",
      contents: [
        {kind: "block", type: "sb_get_current_pose"},
        {kind: "block", type: "sb_get_ball_pose"},
        {kind: "block", type: "sb_ball_pose_known"},
      ],
    },
    {
      kind: "category",
      name: "Poses and field",
      colour: "#D98E2B",
      contents: [
        {kind: "block", type: "sb_make_pose"},
        {kind: "block", type: "sb_pose_attribute"},
        {kind: "block", type: "sb_pose_distance"},
        {kind: "block", type: "sb_pose_direction"},
        {kind: "block", type: "sb_pose_add"},
        {kind: "block", type: "sb_pose_subtract"},
        {kind: "block", type: "sb_pose_from_direction"},
      ],
    },
    {
      kind: "category",
      name: "Control",
      colour: "#FFAB19",
      contents: [{kind: "block", type: "controls_if"}],
    },
    {
      kind: "category",
      name: "Logic",
      colour: "#9966FF",
      contents: [
        {kind: "block", type: "logic_compare"},
        {kind: "block", type: "logic_operation"},
        {kind: "block", type: "logic_negate"},
        {kind: "block", type: "logic_boolean"},
      ],
    },
    {
      kind: "category",
      name: "Math",
      colour: "#59C059",
      contents: [
        {kind: "block", type: "math_number"},
        {kind: "block", type: "math_arithmetic"},
        {kind: "block", type: "math_single"},
      ],
    },
    {
      kind: "category",
      name: "Variables",
      custom: "VARIABLE",
      colour: "#FF6680",
    },
    {
      kind: "category",
      name: "Output",
      colour: "#FF8C1A",
      contents: [
        {kind: "block", type: "text_print"},
        {kind: "block", type: "text"},
      ],
    },
    {
      kind: "category",
      name: "Advanced",
      colour: "#777777",
      contents: [
        {kind: "block", type: "controls_whileUntil"},
        {kind: "block", type: "controls_flow_statements"},
      ],
    },
  ],
};

Blockly.defineBlocksWithJsonArray([
  {
    type: "sb_main",
    message0: "main %1",
    args0: [{type: "input_statement", name: "BODY"}],
    colour: 210,
    tooltip: "Entry point for your simple behavior script",
  },
  {
    type: "time_sleep",
    message0: "sleep %1 seconds",
    args0: [{type: "input_value", name: "SECONDS", check: "Number"}],
    previousStatement: null,
    nextStatement: null,
    colour: 210,
    tooltip: "Pause execution for a short time",
  },
  {
    type: "sb_forever",
    message0: "repeat forever %1",
    args0: [{type: "input_statement", name: "BODY"}],
    previousStatement: null,
    nextStatement: null,
    colour: 210,
    tooltip: "Repeat these blocks until the program is stopped",
  },
  {
    type: "sb_set_head_mode",
    message0: "set head mode %1",
    args0: [
      {
        type: "field_dropdown",
        name: "MODE",
        options: [
          ["track ball", "HEAD_MODE_TRACK_BALL"],
          ["search field features", "HEAD_MODE_SEARCH_FIELD_FEATURES"],
          ["look forward", "HEAD_MODE_LOOK_FORWARD"],
          ["don't move", "HEAD_MODE_DONT_MOVE"],
          ["search ball penalty", "HEAD_MODE_SEARCH_BALL_PENALTY"],
          ["search front", "HEAD_MODE_SEARCH_FRONT"],
        ],
      },
    ],
    previousStatement: null,
    nextStatement: null,
    colour: 160,
    tooltip: "Select the robot head behavior",
  },
  {
    type: "sb_set_goal_pose",
    message0: "set goal pose x %1 y %2 yaw deg %3",
    args0: [
      {type: "input_value", name: "X", check: "Number"},
      {type: "input_value", name: "Y", check: "Number"},
      {type: "input_value", name: "YAW", check: "Number"},
    ],
    previousStatement: null,
    nextStatement: null,
    colour: 160,
    tooltip: "Walk to a goal position on the field",
  },
  {
    type: "sb_set_goal_pose_from_pose",
    message0: "walk to pose %1",
    args0: [{type: "input_value", name: "POSE", check: "Pose"}],
    previousStatement: null,
    nextStatement: null,
    colour: 160,
    tooltip: "Walk to the x and y coordinates of a pose",
  },
  {
    type: "sb_stop",
    message0: "stop walking",
    previousStatement: null,
    nextStatement: null,
    colour: 160,
    tooltip: "Stop the current walking goal",
  },
  {
    type: "sb_kick_ball",
    message0: "kick ball direction deg %1 strength %2",
    args0: [
      {type: "input_value", name: "DIRECTION", check: "Number"},
      {type: "input_value", name: "STRENGTH", check: "Number"},
    ],
    previousStatement: null,
    nextStatement: null,
    colour: 160,
    tooltip: "Kick the ball with map-frame direction and strength",
  },
  {
    type: "sb_get_current_pose",
    message0: "current pose",
    output: "Pose",
    colour: 160,
    tooltip: "Get robot pose with fields x, y, yew_deg",
  },
  {
    type: "sb_get_ball_pose",
    message0: "ball pose",
    output: "Pose",
    colour: 160,
    tooltip: "Get ball pose with fields x and y",
  },
  {
    type: "sb_ball_pose_known",
    message0: "ball position is known",
    output: "Boolean",
    colour: 200,
    tooltip: "True when the robot currently knows where the ball is",
  },
  {
    type: "sb_make_pose",
    message0: "make pose x %1 y %2",
    args0: [
      {type: "input_value", name: "X", check: "Number"},
      {type: "input_value", name: "Y", check: "Number"},
    ],
    output: "Pose",
    colour: 160,
    tooltip: "Create a pose from field coordinates",
  },
  {
    type: "sb_pose_attribute",
    message0: "pose %1 of %2",
    args0: [
      {
        type: "field_dropdown",
        name: "ATTRIBUTE",
        options: [
          ["x", "x"],
          ["y", "y"],
          ["yaw deg", "yew_deg"],
        ],
      },
      {type: "input_value", name: "POSE", check: "Pose"},
    ],
    output: "Number",
    colour: 160,
    tooltip: "Read a field from a pose; ball poses do not have yaw",
  },
  {
    type: "sb_pose_distance",
    message0: "distance from pose %1 to %2",
    args0: [
      {type: "input_value", name: "FIRST", check: "Pose"},
      {type: "input_value", name: "SECOND", check: "Pose"},
    ],
    output: "Number",
    colour: 160,
    tooltip: "Calculate the distance in meters between two poses",
  },
  {
    type: "sb_pose_add",
    message0: "add poses %1 and %2",
    args0: [
      {type: "input_value", name: "FIRST", check: "Pose"},
      {type: "input_value", name: "SECOND", check: "Pose"},
    ],
    output: "Pose",
    colour: 160,
    tooltip: "Add the x and y coordinates of two poses",
  },
  {
    type: "sb_pose_subtract",
    message0: "subtract pose %2 from %1",
    args0: [
      {type: "input_value", name: "FIRST", check: "Pose"},
      {type: "input_value", name: "SECOND", check: "Pose"},
    ],
    output: "Pose",
    colour: 160,
    tooltip: "Subtract the second pose coordinates from the first",
  },
  {
    type: "sb_pose_direction",
    message0: "direction from pose %1 to %2",
    args0: [
      {type: "input_value", name: "FIRST", check: "Pose"},
      {type: "input_value", name: "SECOND", check: "Pose"},
    ],
    output: "Number",
    colour: 160,
    tooltip: "Calculate the direction in degrees from one pose to another",
  },
  {
    type: "sb_pose_from_direction",
    message0: "point from pose %1 distance %2 direction deg %3",
    args0: [
      {type: "input_value", name: "ORIGIN", check: "Pose"},
      {type: "input_value", name: "DISTANCE", check: "Number"},
      {type: "input_value", name: "DIRECTION", check: "Number"},
    ],
    output: "Pose",
    colour: 160,
    tooltip: "Create a pose at a distance and direction from another pose",
  },
]);

const pythonGenerator = Blockly.Python;

pythonGenerator.forBlock["sb_main"] = function (block, generator) {
  const body = generator.statementToCode(block, "BODY");
  const content = body || "    pass\n";
  return `from time import sleep\nfrom bitbots_body_behavior.simple_behavior import *\n\n\ndef main():\n${content}`;
};

pythonGenerator.forBlock["time_sleep"] = function (block, generator) {
  const seconds = generator.valueToCode(block, "SECONDS", pythonGenerator.ORDER_NONE) || "0.1";
  return `sleep(${seconds})\n`;
};

pythonGenerator.forBlock["sb_forever"] = function (block, generator) {
  const body = generator.statementToCode(block, "BODY") || "    pass\n";
  return `while True:\n${body}`;
};

pythonGenerator.forBlock["sb_set_head_mode"] = function (block) {
  const mode = block.getFieldValue("MODE");
  return `set_head_mode(${mode})\n`;
};

pythonGenerator.forBlock["sb_set_goal_pose"] = function (block, generator) {
  const x = generator.valueToCode(block, "X", pythonGenerator.ORDER_NONE) || "0.0";
  const y = generator.valueToCode(block, "Y", pythonGenerator.ORDER_NONE) || "0.0";
  const yaw = generator.valueToCode(block, "YAW", pythonGenerator.ORDER_NONE) || "0.0";
  return `set_goal_pose(${x}, ${y}, ${yaw})\n`;
};

pythonGenerator.forBlock["sb_set_goal_pose_from_pose"] = function (block, generator) {
  const pose = generator.valueToCode(block, "POSE", pythonGenerator.ORDER_NONE) || "make_pose(0.0, 0.0)";
  return `set_goal_pose_from_pose(${pose})\n`;
};

pythonGenerator.forBlock["sb_stop"] = function () {
  return "stop()\n";
};

pythonGenerator.forBlock["sb_kick_ball"] = function (block, generator) {
  const direction = generator.valueToCode(block, "DIRECTION", pythonGenerator.ORDER_NONE) || "0.0";
  const strength = generator.valueToCode(block, "STRENGTH", pythonGenerator.ORDER_NONE) || "3.0";
  return `kick_ball(${direction}, ${strength})\n`;
};

pythonGenerator.forBlock["sb_get_current_pose"] = function () {
  return ["get_current_pose()", pythonGenerator.ORDER_FUNCTION_CALL];
};

pythonGenerator.forBlock["sb_get_ball_pose"] = function () {
  return ["get_ball_pose()", pythonGenerator.ORDER_FUNCTION_CALL];
};

pythonGenerator.forBlock["sb_ball_pose_known"] = function () {
  return ["get_ball_pose() is not None", pythonGenerator.ORDER_RELATIONAL];
};

pythonGenerator.forBlock["sb_make_pose"] = function (block, generator) {
  const x = generator.valueToCode(block, "X", pythonGenerator.ORDER_NONE) || "0.0";
  const y = generator.valueToCode(block, "Y", pythonGenerator.ORDER_NONE) || "0.0";
  return [`make_pose(${x}, ${y})`, pythonGenerator.ORDER_FUNCTION_CALL];
};

pythonGenerator.forBlock["sb_pose_attribute"] = function (block, generator) {
  const attribute = block.getFieldValue("ATTRIBUTE");
  const pose = generator.valueToCode(block, "POSE", pythonGenerator.ORDER_MEMBER) || "get_current_pose()";
  return [`(${pose}).${attribute}`, pythonGenerator.ORDER_MEMBER];
};

function poseOperationGenerator(functionName, resultOrder) {
  return function (block, generator) {
    const first = generator.valueToCode(block, "FIRST", pythonGenerator.ORDER_NONE) || "make_pose(0.0, 0.0)";
    const second = generator.valueToCode(block, "SECOND", pythonGenerator.ORDER_NONE) || "make_pose(0.0, 0.0)";
    return [`${functionName}(${first}, ${second})`, resultOrder];
  };
}

pythonGenerator.forBlock["sb_pose_distance"] = poseOperationGenerator(
  "pose_distance",
  pythonGenerator.ORDER_FUNCTION_CALL,
);
pythonGenerator.forBlock["sb_pose_add"] = poseOperationGenerator("pose_add", pythonGenerator.ORDER_FUNCTION_CALL);
pythonGenerator.forBlock["sb_pose_subtract"] = poseOperationGenerator(
  "pose_subtract",
  pythonGenerator.ORDER_FUNCTION_CALL,
);
pythonGenerator.forBlock["sb_pose_direction"] = poseOperationGenerator(
  "pose_direction",
  pythonGenerator.ORDER_FUNCTION_CALL,
);

pythonGenerator.forBlock["sb_pose_from_direction"] = function (block, generator) {
  const origin = generator.valueToCode(block, "ORIGIN", pythonGenerator.ORDER_NONE) || "make_pose(0.0, 0.0)";
  const distance = generator.valueToCode(block, "DISTANCE", pythonGenerator.ORDER_NONE) || "0.0";
  const direction = generator.valueToCode(block, "DIRECTION", pythonGenerator.ORDER_NONE) || "0.0";
  return [`pose_from_direction(${origin}, ${distance}, ${direction})`, pythonGenerator.ORDER_FUNCTION_CALL];
};

function buildDefaultWorkspaceXml() {
  return `
<xml xmlns="https://developers.google.com/blockly/xml">
  <block type="sb_main" x="20" y="20">
    <statement name="BODY">
      <block type="sb_set_head_mode">
        <field name="MODE">HEAD_MODE_TRACK_BALL</field>
        <next>
          <block type="sb_forever">
            <statement name="BODY">
              <block type="controls_if">
                <value name="IF0">
                  <block type="sb_ball_pose_known"></block>
                </value>
                <statement name="DO0">
                  <block type="variables_set">
                    <field name="VAR">ball_pose</field>
                    <value name="VALUE">
                      <block type="sb_get_ball_pose"></block>
                    </value>
                    <next>
                      <block type="sb_set_goal_pose">
                        <value name="X">
                          <block type="sb_pose_attribute">
                            <field name="ATTRIBUTE">x</field>
                            <value name="POSE">
                              <block type="variables_get">
                                <field name="VAR">ball_pose</field>
                              </block>
                            </value>
                          </block>
                        </value>
                        <value name="Y">
                          <block type="sb_pose_attribute">
                            <field name="ATTRIBUTE">y</field>
                            <value name="POSE">
                              <block type="variables_get">
                                <field name="VAR">ball_pose</field>
                              </block>
                            </value>
                          </block>
                        </value>
                        <value name="YAW">
                          <shadow type="math_number">
                            <field name="NUM">0</field>
                          </shadow>
                        </value>
                      </block>
                    </next>
                  </block>
                </statement>
                <next>
                  <block type="time_sleep">
                    <value name="SECONDS">
                      <shadow type="math_number">
                        <field name="NUM">0.1</field>
                      </shadow>
                    </value>
                  </block>
                </next>
              </block>
            </statement>
          </block>
        </next>
      </block>
    </statement>
  </block>
</xml>`;
}

const WORKSPACE_STORAGE_KEY = "bitbots-simple-behavior-blockly-workspace";

function loadWorkspace(workspace) {
  try {
    const savedWorkspace = window.localStorage.getItem(WORKSPACE_STORAGE_KEY);
    if (savedWorkspace) {
      Blockly.Xml.domToWorkspace(Blockly.utils.xml.textToDom(savedWorkspace), workspace);
      return;
    }
  } catch (error) {
    console.warn("Could not restore the saved Blockly workspace.", error);
  }

  const defaultXml = Blockly.utils.xml.textToDom(buildDefaultWorkspaceXml());
  Blockly.Xml.domToWorkspace(defaultXml, workspace);
}

function saveWorkspace(workspace) {
  try {
    const workspaceXml = Blockly.Xml.workspaceToDom(workspace);
    window.localStorage.setItem(WORKSPACE_STORAGE_KEY, Blockly.Xml.domToText(workspaceXml));
  } catch (error) {
    console.warn("Could not save the Blockly workspace.", error);
  }
}

function initializeBlocklyPage() {
  const workspace = Blockly.inject("blocklyDiv", {
    toolbox: SIMPLE_BEHAVIOR_TOOLBOX,
    grid: {spacing: 20, length: 3, colour: "#ccc", snap: true},
    zoom: {controls: true, wheel: true, startScale: 1.0},
    trashcan: true,
  });

  loadWorkspace(workspace);

  const output = document.getElementById("pythonOutput");
  const runStatus = document.getElementById("runStatus");
  const updateCode = () => {
    const raw = Blockly.Python.workspaceToCode(workspace);
    output.value = raw.trim() ? `${raw}\n` : "";
    if (runStatus) {
      runStatus.textContent = "";
    }
  };

  workspace.addChangeListener((event) => {
    updateCode();
    if (!event.isUiEvent) {
      saveWorkspace(workspace);
    }
  });
  updateCode();

  document.getElementById("copyButton").addEventListener("click", async () => {
    await navigator.clipboard.writeText(output.value);
  });

  document.getElementById("downloadButton").addEventListener("click", () => {
    const blob = new Blob([output.value], {type: "text/x-python;charset=utf-8"});
    const link = document.createElement("a");
    link.href = URL.createObjectURL(blob);
    link.download = "simple_behavior_script.py";
    link.click();
    URL.revokeObjectURL(link.href);
  });

  const runButton = document.getElementById("runButton");
  const runEndpoint = document.getElementById("runEndpoint");
  if (runButton && runEndpoint && runStatus) {
    runButton.addEventListener("click", async () => {
      const endpoint = runEndpoint.value.trim();
      if (!endpoint) {
        runStatus.textContent = "Set a run endpoint first.";
        return;
      }

      if (!output.value.trim()) {
        runStatus.textContent = "Generate a script first.";
        return;
      }

      runStatus.textContent = "Sending script...";
      try {
        const response = await fetch(endpoint, {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({
            name: "simple_behavior_blockly.py",
            script: output.value,
          }),
        });
        const result = await response.json();
        if (response.ok && result.ok) {
          runStatus.textContent = "Script submitted successfully.";
        } else {
          runStatus.textContent = `Submit failed: ${result.error || response.statusText}`;
        }
      } catch (error) {
        runStatus.textContent = `Submit failed: ${error}`;
      }
    });
  }
}

window.addEventListener("DOMContentLoaded", initializeBlocklyPage);