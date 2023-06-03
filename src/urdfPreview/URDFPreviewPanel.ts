import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';
import * as urdf from '@polyhobbyist/babylon_ros';
import * as ColladaFileLoader from '@polyhobbyist/babylon-collada-loader';
import * as GUI from 'babylonjs-gui';

// Get access to the VS Code API from within the webview context
const vscode = acquireVsCodeApi();

const canvas = document.getElementById("renderCanvas") as HTMLCanvasElement; // Get the canvas element
const engine = new BABYLON.Engine(canvas, true); // Generate the BABYLON 3D engine
let scene : BABYLON.Scene | undefined = undefined;

let currentRobot : urdf.Robot | undefined = undefined;

let ground : BABYLON.GroundMesh | undefined = undefined;
let camera : BABYLON.ArcRotateCamera | undefined = undefined;
let statusLabel = new GUI.TextBlock();

let readyToRender : Boolean = false;

let axisList : BABYLON.PositionGizmo[] = [];

function addAxisToTransform(scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer, transform : BABYLON.TransformNode | undefined) {
  if (transform) {
    let axis = new BABYLON.PositionGizmo(layer);
    axis.scaleRatio = 0.5
    axis.attachedNode = transform;
    axisList.push(axis);

    let drag = () => {
      if (transform) {
        statusLabel.text = transform.name + 
        "\nX: " + transform.position.x.toFixed(6) + 
        "\nY: " + transform.position.y.toFixed(6) + 
        "\nZ: " + transform.position.z.toFixed(6);
        statusLabel.linkOffsetY = -100;
      statusLabel.linkWithMesh(transform);
      }
    }

    axis.xGizmo.dragBehavior.onDragObservable.add(drag);
    axis.yGizmo.dragBehavior.onDragObservable.add(drag);
    axis.zGizmo.dragBehavior.onDragObservable.add(drag);
      
  }
}

function toggleAxisOnRobot(scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer, robot : urdf.Robot) {

  if (axisList.length == 0) {
    robot.joints.forEach((j) => {
      addAxisToTransform(scene, layer, j.transform);
    });
    robot.links.forEach((l) => {
      l.visuals.forEach((v) => {
        addAxisToTransform(scene, layer, l.transform);
      });
    });
  } else {
    axisList.forEach((a) => {
      a.dispose();
    });
    axisList = [];
  }
}

let rotationGizmos : BABYLON.RotationGizmo[] = [];

function addRotationToTransform(scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer, transform : BABYLON.TransformNode | undefined) {
  if (transform) {
    let rotationGizmo = new BABYLON.RotationGizmo(layer);
    rotationGizmo.scaleRatio = 0.5
    rotationGizmo.attachedNode = transform;
    rotationGizmos.push(rotationGizmo);

    let drag = () => {
      if (transform) {
        statusLabel.text = transform.name + 
        "\nR:" + transform.rotation.x.toFixed(6) + 
        "\nP:" + transform.rotation.y.toFixed(6) + 
        "\nY:" + transform.rotation.z.toFixed(6);
        statusLabel.linkOffsetY = -100;
      statusLabel.linkWithMesh(transform);
      }
    }

    rotationGizmo.xGizmo.dragBehavior.onDragObservable.add(drag);
    rotationGizmo.yGizmo.dragBehavior.onDragObservable.add(drag);
    rotationGizmo.zGizmo.dragBehavior.onDragObservable.add(drag);
  }

}

function toggleAxisRotationOnRobot(ui: GUI.AdvancedDynamicTexture, scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer, robot : urdf.Robot) {
  if (rotationGizmos.length == 0) {
    robot.joints.forEach((j) => {
      addRotationToTransform(scene, layer, j.transform);
    });

    robot.links.forEach((l) => {
      l.visuals.forEach((v) => {
        addRotationToTransform(scene, layer, v.transform);
      });
    });
  } else {
    rotationGizmos.forEach((a) => {
      a.dispose();
    });
    rotationGizmos = [];
  }
}


var createScene = async function () {
  scene = new BABYLON.Scene(engine);
  if (BABYLON.SceneLoader) {
    //Add this loader into the register plugin
    BABYLON.SceneLoader.RegisterPlugin(new ColladaFileLoader.DAEFileLoader());
  }

  // This creates a basic Babylon Scene object (non-mesh)
    // Create a default ground and skybox.
  const environment = scene.createDefaultEnvironment({
    createGround: true,
    createSkybox: false,
    enableGroundMirror: true,
    groundMirrorSizeRatio: 0.15
  });

  scene.useRightHandedSystem = true;
  scene.clearColor = BABYLON.Color4.FromColor3(BABYLON.Color3.Black());

  
  // This creates and positions a free camera (non-mesh)
  camera = new BABYLON.ArcRotateCamera("camera1", - Math.PI / 3, 5 * Math.PI / 12, 1, new BABYLON.Vector3(0, 0, 0), scene);
  camera.wheelDeltaPercentage = 0.01;
  camera.minZ = 0.1;

  const light = new BABYLON.HemisphericLight("light", new BABYLON.Vector3(0, 1, 0), scene);

  // This attaches the camera to the canvas
  camera.attachControl(canvas, true);

  var groundMaterial = new Materials.GridMaterial("groundMaterial", scene);
  groundMaterial.majorUnitFrequency = 5;
  groundMaterial.minorUnitVisibility = 0.5;
  groundMaterial.gridRatio = 1;
  groundMaterial.opacity = 0.8;
  groundMaterial.useMaxLine = true;
  groundMaterial.lineColor = BABYLON.Color3.Green();
  groundMaterial.mainColor = BABYLON.Color3.Green();

  ground = BABYLON.MeshBuilder.CreateGround("ground", {width: 50, height: 50}, scene);
  ground.material = groundMaterial;
  ground.isPickable = false;


  return scene;
};

function createUI(scene : BABYLON.Scene) {
  var advancedTexture = GUI.AdvancedDynamicTexture.CreateFullscreenUI("UI");

  statusLabel.color = "white";
  statusLabel.textHorizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_LEFT;
  statusLabel.horizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_CENTER;
  statusLabel.resizeToFit = true;
  statusLabel.outlineColor = "green";
  statusLabel.outlineWidth = 2.0;
  advancedTexture.addControl(statusLabel);

  var toolbar = new GUI.StackPanel();
  toolbar.paddingTop = "10px";
  toolbar.paddingLeft = "10px";
  toolbar.width = "300px";
  toolbar.height = "50px";
  toolbar.fontSize = "14px";
  toolbar.horizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_LEFT;
  toolbar.verticalAlignment = GUI.Control.VERTICAL_ALIGNMENT_TOP;
  toolbar.isVertical = false;
  advancedTexture.addControl(toolbar);


  var utilLayer = new BABYLON.UtilityLayerRenderer(scene);

  const gizmoManager = new BABYLON.GizmoManager(scene);
  gizmoManager.usePointerToAttachGizmos = false;

  var button = GUI.Button.CreateSimpleButton("axisButton", "Axis");
  button.width = 0.2;
  button.height = "40px";
  button.color = "white";
  button.background = "green";
  button.onPointerClickObservable.add(function() {

    toggleAxisOnRobot(scene, utilLayer, currentRobot);
  });
  toolbar.addControl(button);

  var buttonRotate = GUI.Button.CreateSimpleButton("rotatioButton", "Rotation");
  buttonRotate.width = 0.2;
  buttonRotate.height = "40px";
  buttonRotate.color = "white";
  buttonRotate.background = "green";
  buttonRotate.onPointerClickObservable.add(function() {
    toggleAxisRotationOnRobot(advancedTexture, scene, utilLayer, currentRobot);
  });  

  toolbar.addControl(buttonRotate);

}

async function applyURDF(urdfText) {
  try {
    if (currentRobot) {
      currentRobot.dispose();
      currentRobot = undefined;
    }

    vscode.postMessage({
      command: "trace",
      text: `loading urdf`,
    });


    currentRobot = await urdf.deserializeUrdfToRobot(urdfText);
    currentRobot.create(scene);

    vscode.postMessage({
      command: "trace",
      text: `loaded urdf`,
    });
  } catch (err) {
    
    vscode.postMessage({
      command: "error",
      text: `Could not render URDF due to: ${err}\n${err.stack}`,
    });
  }
}

// Main function that gets executed once the webview DOM loads
async function main() {

  scene = await createScene();
  createUI(scene);

  window.addEventListener('message', event => {
    const message = event.data; // The JSON data our extension sent
    switch (message.command) {
        case 'urdf':
        applyURDF(message.urdf);
        break;
        case 'previewFile':
        vscode.setState({previewFile: message.previewFile});
        break;
        case 'colors':
          camera.radius = message.cameraRadius;
          scene.clearColor = BABYLON.Color4.FromHexString(message.backgroundColor);
          let gm = ground.material as Materials.GridMaterial;
          gm.lineColor = BABYLON.Color3.FromHexString(message.gridLineColor);
          gm.mainColor = BABYLON.Color3.FromHexString(message.gridMainColor);
          gm.minorUnitVisibility = parseFloat(message.gridMinorOpacity);

          readyToRender = true;
          break;
    }
  });
  
  engine.runRenderLoop(function () {
    if (scene != undefined && readyToRender) {
      scene.render();
    }
  });
  
  engine.resize();
  
  window.addEventListener("resize", function () {
      engine.resize();
  });  
  
  
}

  // Just like a regular webpage we need to wait for the webview
  // DOM to load before we can reference any of the HTML elements
  // or toolkit components
  window.addEventListener("load", main);
  
