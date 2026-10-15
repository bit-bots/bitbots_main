// Keep these presets synchronized with bitbots_parameter_blackboard/config/fields/*/config.yaml.
const FIELD_PRESETS = {
  demo: {length: 20.0, width: 20.0, padding: 0.1, penaltyDepth: 0.75, goalWidth: 2.25, goalDepth: 0.6},
  german_open_2024: {length: 9.0, width: 6.0, padding: 1.0, penaltyDepth: 2.0, goalWidth: 2.6, goalDepth: 0.6},
  german_open_2026_medium: {
    length: 8.93,
    width: 5.94,
    padding: 1.0,
    penaltyDepth: 1.93,
    goalWidth: 2.5,
    goalDepth: 0.75,
  },
  hsl_kid: {length: 9.0, width: 6.0, padding: 1.0, penaltyDepth: 2.0, goalWidth: 1.85, goalDepth: 0.75},
  ifa26: {length: 7.45, width: 5.01, padding: 0.8, penaltyDepth: 1.65, goalWidth: 1.87, goalDepth: 0.75},
  labor: {length: 5.0, width: 3.87, padding: 0.16, penaltyDepth: 1.0, goalWidth: 1.2, goalDepth: 0.16},
  middle_division_2026: {
    length: 14.0,
    width: 9.0,
    padding: 1.0,
    penaltyDepth: 3.0,
    goalWidth: 2.6,
    goalDepth: 0.75,
  },
  small_division_2026: {
    length: 9.0,
    width: 6.0,
    padding: 1.0,
    penaltyDepth: 2.0,
    goalWidth: 1.8,
    goalDepth: 0.5,
  },
  spl: {length: 9.0, width: 6.0, padding: 0.7, penaltyDepth: 1.65, goalWidth: 1.5, goalDepth: 0.5},
  webots: {length: 9.0, width: 6.0, padding: 1.0, penaltyDepth: 2.0, goalWidth: 2.6, goalDepth: 0.6},
};

const SVG_NS = "http://www.w3.org/2000/svg";

function createSvgElement(tagName, attributes = {}, text = "") {
  const element = document.createElementNS(SVG_NS, tagName);
  for (const [name, value] of Object.entries(attributes)) {
    element.setAttribute(name, value);
  }
  element.textContent = text;
  return element;
}

function initializeFieldReference() {
  const diagram = document.getElementById("fieldDiagram");
  const presetSelect = document.getElementById("fieldPreset");
  const readout = document.getElementById("fieldReadout");
  if (!diagram || !presetSelect || !readout) {
    return;
  }

  for (const presetName of Object.keys(FIELD_PRESETS)) {
    const option = document.createElement("option");
    option.value = presetName;
    option.textContent = presetName;
    option.selected = presetName === "small_division_2026";
    presetSelect.append(option);
  }

  let field = FIELD_PRESETS.small_division_2026;
  let point = {x: 0, y: 0};
  let transform = null;
  let pointer = null;

  const updatePointer = () => {
    if (!pointer || !transform) {
      return;
    }
    pointer.setAttribute("transform", `translate(${transform.toX(point.x)} ${transform.toY(point.y)})`);
    readout.textContent = `x = ${point.x.toFixed(2)} m, y = ${point.y.toFixed(2)} m`;
    pointer.setAttribute("aria-label", `Selected point x ${point.x.toFixed(2)} meters, y ${point.y.toFixed(2)} meters`);
  };

  const setPoint = (x, y) => {
    const xLimit = field.length / 2 + field.padding;
    const yLimit = field.width / 2 + field.padding;
    point = {
      x: Math.max(-xLimit, Math.min(xLimit, x)),
      y: Math.max(-yLimit, Math.min(yLimit, y)),
    };
    updatePointer();
  };

  const drawField = () => {
    diagram.replaceChildren();
    const canvasWidth = 800;
    const canvasHeight = 520;
    const margin = 42;
    const outerLength = field.length + 2 * Math.max(field.padding, field.goalDepth);
    const outerWidth = field.width + 2 * field.padding;
    const scale = Math.min((canvasWidth - 2 * margin) / outerLength, (canvasHeight - 2 * margin) / outerWidth);
    const centerX = canvasWidth / 2;
    const centerY = canvasHeight / 2;
    const toX = (x) => centerX + x * scale;
    const toY = (y) => centerY - y * scale;
    transform = {toX, toY, scale};

    const paddingLeft = toX(-field.length / 2 - field.padding);
    const paddingTop = toY(field.width / 2 + field.padding);
    diagram.append(
      createSvgElement("rect", {
        x: paddingLeft,
        y: paddingTop,
        width: (field.length + 2 * field.padding) * scale,
        height: (field.width + 2 * field.padding) * scale,
        fill: "#238c4b",
      })
    );

    const lineStyle = {fill: "none", stroke: "white", "stroke-width": 3};
    diagram.append(
      createSvgElement("rect", {
        x: toX(-field.length / 2),
        y: toY(field.width / 2),
        width: field.length * scale,
        height: field.width * scale,
        ...lineStyle,
      }),
      createSvgElement("line", {
        x1: toX(0),
        y1: toY(field.width / 2),
        x2: toX(0),
        y2: toY(-field.width / 2),
        ...lineStyle,
      }),
      createSvgElement("circle", {cx: toX(0), cy: toY(0), r: 5, fill: "white"})
    );

    for (const side of [-1, 1]) {
      const goalLineX = side * field.length / 2;
      const goalBackX = side * (field.length / 2 + field.goalDepth);
      diagram.append(
        createSvgElement("rect", {
          x: Math.min(toX(goalLineX), toX(goalBackX)),
          y: toY(field.goalWidth / 2),
          width: field.goalDepth * scale,
          height: field.goalWidth * scale,
          ...lineStyle,
        }),
        createSvgElement("line", {
          x1: toX(side * (field.length / 2 - field.penaltyDepth)),
          y1: toY(field.goalWidth / 2),
          x2: toX(side * (field.length / 2 - field.penaltyDepth)),
          y2: toY(-field.goalWidth / 2),
          stroke: "white",
          "stroke-width": 2,
          "stroke-dasharray": "7 5",
        })
      );
    }

    const axisStyle = {stroke: "#ffe14d", "stroke-width": 3};
    diagram.append(
      createSvgElement("line", {x1: toX(0), y1: toY(0), x2: toX(field.length / 4), y2: toY(0), ...axisStyle}),
      createSvgElement("line", {x1: toX(0), y1: toY(0), x2: toX(0), y2: toY(field.width / 4), ...axisStyle}),
      createSvgElement("text", {x: toX(field.length / 4) + 8, y: toY(0) + 5, fill: "#ffe14d"}, "+x"),
      createSvgElement("text", {x: toX(0) + 8, y: toY(field.width / 4) - 5, fill: "#ffe14d"}, "+y"),
      createSvgElement("text", {x: toX(-field.length / 2), y: toY(-field.width / 2) + 25, fill: "white"}, "own goal"),
      createSvgElement(
        "text",
        {x: toX(field.length / 2), y: toY(-field.width / 2) + 25, fill: "white", "text-anchor": "end"},
        "opponent goal"
      )
    );

    pointer = createSvgElement("g", {id: "fieldPointer", tabindex: "0", role: "slider"});
    pointer.append(
      createSvgElement("line", {x1: -14, y1: 0, x2: 14, y2: 0, stroke: "#111", "stroke-width": 2}),
      createSvgElement("line", {x1: 0, y1: -14, x2: 0, y2: 14, stroke: "#111", "stroke-width": 2}),
      createSvgElement("circle", {cx: 0, cy: 0, r: 9, fill: "#ffe14d", stroke: "white", "stroke-width": 3})
    );
    diagram.append(pointer);
    pointer.addEventListener("keydown", (event) => {
      const movement = {
        ArrowLeft: [-0.1, 0],
        ArrowRight: [0.1, 0],
        ArrowDown: [0, -0.1],
        ArrowUp: [0, 0.1],
      }[event.key];
      if (movement) {
        event.preventDefault();
        setPoint(point.x + movement[0], point.y + movement[1]);
      }
    });
    updatePointer();
  };

  const movePointFromEvent = (event) => {
    const svgPoint = diagram.createSVGPoint();
    svgPoint.x = event.clientX;
    svgPoint.y = event.clientY;
    const localPoint = svgPoint.matrixTransform(diagram.getScreenCTM().inverse());
    setPoint((localPoint.x - 400) / transform.scale, (260 - localPoint.y) / transform.scale);
  };

  diagram.addEventListener("pointerdown", (event) => {
    diagram.setPointerCapture(event.pointerId);
    movePointFromEvent(event);
  });
  diagram.addEventListener("pointermove", (event) => {
    if (diagram.hasPointerCapture(event.pointerId)) {
      movePointFromEvent(event);
    }
  });

  presetSelect.addEventListener("change", () => {
    field = FIELD_PRESETS[presetSelect.value];
    point = {x: 0, y: 0};
    drawField();
  });

  drawField();
}

window.addEventListener("DOMContentLoaded", initializeFieldReference);