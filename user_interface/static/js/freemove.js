/**
 * Free Move tab: drag-to-jog TCP control for the UR10e and Kawasaki arms.
 *
 * Scene layout: robot, ghosts and targets are ALL direct children of `scene` --
 * no wrapper group, no rotation anywhere in the graph. ROS is Z-up and URDFLoader
 * loads links "without frame transforms" (its own stated convention), so every
 * object's .position/.quaternion IS the ROS `world`-frame pose already; sent to the
 * backend as-is.
 *
 * An earlier version wrapped everything in a group rotated -90 deg about X to make
 * a Z-up robot look upright under three.js's normally Y-up camera. That was the bug
 * behind "dragging what looks like one axis moves a different one on the real robot":
 * TransformControls' world-space gizmo arrows follow the SCENE's world axes, which
 * were tilted relative to ROS's by that rotation, so the on-screen red/green/blue
 * arrows did not correspond to ROS X/Y/Z at all. The fix is the standard three.js
 * way to render a Z-up scene: leave the scene graph alone and set `camera.up` to
 * (0,0,1) instead -- OrbitControls reads camera.up for its own orbiting math, so the
 * view still orbits sensibly, and now literally everything (camera included) shares
 * one frame with no conversion anywhere.
 */

// Ghost tint is intentionally the SAME orange for both arms, distinct from either arm's
// own identity color below -- the real (grey/metal, untouched) robot and the ghost
// preview need to read apart at a glance, and per-arm ghost colors (orange vs blue)
// made the UR ghost blend into the real UR arm's own bluish plastic.
const FM_GHOST_COLOR = 0xf97316;
const FM_ARMS = {
    ur: { label: "UR10e", color: 0x3b82f6, tcpLink: "ur10e_tool0" },
    kawasaki: { label: "Kawasaki RS005L", color: 0xf97316, tcpLink: "link6" },
};

// SICK optical frames, both rigidly (fixed-joint-only) mounted relative to their
// arm's TCP link -- confirmed by walking the URDF's joint chain: every joint between
// ur10e_tool0/link6 and these frames is `fixed`. That means "camera pose" is just
// "TCP pose composed with a constant offset", computable straight from the target
// ball with no IK round trip -- so the camera direction can be shown live while
// dragging, not just after a plan comes back.
const FM_CAMERA_LINK = { ur: "ur10e_sick_optical_frame", kawasaki: "kawasaki_sick_optical_frame" };

// Links that are actually PART OF the moving arm (walked from the URDF's joint parent/
// child chain, 2026-09-09) -- everything else in the cell (the UR's mounting table +
// chassis legs, the Kawasaki AGV's mobile base/wheels/casters) is fixed to `world` or
// only shares the rail joint, not something an operator would call "the robot". The
// ghost only tints/shows these; every other mesh in the loaded cell is hidden on the
// ghost so environmental structure never reads as part of the preview.
const FM_ROBOT_LINK_NAMES = {
    ur: new Set([
        "ur10e_robot_mount", "ur10e_base_link", "ur10e_base_link_inertia", "ur10e_base",
        "ur10e_shoulder_link", "ur10e_upper_arm_link", "ur10e_forearm_link",
        "ur10e_wrist_1_link", "ur10e_wrist_2_link", "ur10e_wrist_3_link", "ur10e_tool0",
        "ur10e_ft_frame", "ur10e_cable_channel_flange", "ur10e_cable_channel",
        "ur10e_flange", "ur10e_sick_camera", "ur10e_depth_frame",
        "ur10e_depth_optical_frame", "ur10e_rgb_frame", "ur10e_rgb_optical_frame",
        "ur10e_sick_optical_frame",
    ]),
    kawasaki: new Set([
        "base_link", "link1", "link2", "link3", "kawa_cable_channel", "link4", "link5",
        "link6", "link7", "camera_link", "camera_rgb_frame", "camera_rgb_optic_frame",
        "kawasaki_sick_optical_frame",
    ]),
};

// Must match free_move.py's UR_JOINT_NAMES / KAWASAKI_JOINT_NAMES -- the joint-angle
// slider panel builds one slider per name here, bounded by that joint's real URDF
// limit (read straight off the loaded model, not hardcoded).
const FM_JOINT_NAMES = {
    ur: [
        "ur10e_base_to_robot_mount", "ur10e_shoulder_pan_joint", "ur10e_shoulder_lift_joint",
        "ur10e_elbow_joint", "ur10e_wrist_1_joint", "ur10e_wrist_2_joint", "ur10e_wrist_3_joint",
    ],
    kawasaki: ["world_to_agv", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
};
const FM_JOINT_LABELS = {
    ur10e_base_to_robot_mount: "rail", ur10e_shoulder_pan_joint: "shoulder pan",
    ur10e_shoulder_lift_joint: "shoulder lift", ur10e_elbow_joint: "elbow",
    ur10e_wrist_1_joint: "wrist 1", ur10e_wrist_2_joint: "wrist 2", ur10e_wrist_3_joint: "wrist 3",
    world_to_agv: "AGV rail", joint1: "joint 1", joint2: "joint 2", joint3: "joint 3",
    joint4: "joint 4", joint5: "joint 5", joint6: "joint 6",
};
const fmJointGoal = { ur: {}, kawasaki: {} };
const fmJointPlanValidFor = { ur: null, kawasaki: null };
// While true (the default), that arm's sliders passively follow the real robot's
// live joint state every frame (see fmSyncTrackedSliders, called from
// fmStepJointInterpolation). Dragging a slider turns tracking off for that arm so the
// user's in-progress goal isn't yanked out from under them; the "Sync sliders to
// current pose" button (fmSyncJointsToCurrent) turns it back on, and so does a
// successful Execute (Cartesian or joint-space) once the move finishes.
const fmJointTracking = { ur: true, kawasaki: true };
// The last trajectory a Plan returned, kept so Execute can re-play the SAME points
// through the ghost once the real move actually starts (see fmAnimatePlan / the
// "executing" branch of freemove_execute_status) -- the ghost used to only animate
// once, during the Plan preview, and never again, so by the time a slow real move
// (Kawasaki's default velocity scale is 0.02) was still underway the ghost had long
// since finished and sat frozen at the final pose: ghost and real robot visibly
// running different things. fmAnimGeneration invalidates any still-running animation
// from a previous Plan/Execute so an old and a new step() chain never fight over the
// same ghost.
const fmLastTrajectory = { ur: null, kawasaki: null };
const fmAnimGeneration = { ur: 0, kawasaki: 0 };

let fmBuilt = false;
let fmScene, fmCamera, fmRenderer, fmOrbit, fmGrid;
let fmRobot = null;
const fmGhosts = {};
const fmTargets = {};
// One TransformControls per arm; mode ("translate"/"rotate") switched instantly via
// G/R keys or the Move/Rotate buttons -- see the comment where these are created for
// why there isn't one instance of each running side by side.
const fmGizmos = {};
const fmGizmoMode = { ur: "translate", kawasaki: "translate" };
// TCP-local {position, quaternion} of each arm's camera, computed once from the
// loaded model (see fmComputeCameraOffsets) and reused for every subsequent drag.
const fmCameraOffsets = {};
const fmCameraArrows = {};
const fmCameraTrails = {};
const fmPlanValidFor = { ur: null, kawasaki: null };
const fmArmReady = { ur: false, kawasaki: false };
let fmActive = false;
let fmLogAutoScroll = true;
const FM_MAX_LOG_ENTRIES = 300;

function initFreeMove() {
    if (fmBuilt) return;
    if (typeof THREE === "undefined" || typeof URDFLoader === "undefined") {
        fmSetBanner("Failed to load the 3D viewer libraries (check internet/CDN access).", true);
        return;
    }
    fmBuilt = true;

    const viewport = document.getElementById("freemove-viewport");

    fmScene = new THREE.Scene();
    fmScene.background = new THREE.Color(0x0b1220);

    fmCamera = new THREE.PerspectiveCamera(
        50, viewport.clientWidth / Math.max(1, viewport.clientHeight), 0.01, 50);
    fmCamera.up.set(0, 0, 1); // ROS is Z-up; see the file header comment.
    fmCamera.position.set(2.4, 1.8, 2.4);

    fmRenderer = new THREE.WebGLRenderer({ antialias: true });
    fmRenderer.setSize(viewport.clientWidth, viewport.clientHeight);
    fmRenderer.setPixelRatio(window.devicePixelRatio || 1);
    viewport.appendChild(fmRenderer.domElement);

    fmScene.add(new THREE.AmbientLight(0xffffff, 0.55));
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.7);
    dirLight.position.set(3, 2, 5); // roughly overhead in this Z-up scene
    fmScene.add(dirLight);
    fmGrid = new THREE.GridHelper(4, 40, 0x334155, 0x1e293b);
    fmGrid.rotation.x = Math.PI / 2; // GridHelper lies in XZ by default; make it XY (Z=0 floor)
    fmScene.add(fmGrid);

    fmOrbit = new THREE.OrbitControls(fmCamera, fmRenderer.domElement);
    fmOrbit.target.set(0, 0, 0.6);
    fmOrbit.update();

    Object.keys(FM_ARMS).forEach((arm) => {
        const cfg = FM_ARMS[arm];

        const target = new THREE.Mesh(
            new THREE.SphereGeometry(0.03, 24, 16),
            new THREE.MeshBasicMaterial({ color: cfg.color }));
        target.visible = false;
        fmScene.add(target);
        fmTargets[arm] = target;

        // Shows where this arm's SICK camera would be and which way it would look
        // if the operator executed the current goal ball -- rigid offset from the
        // target, so it updates live while dragging with no IK round trip needed.
        const camArrow = new THREE.ArrowHelper(
            new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 0.3, cfg.color, 0.09, 0.06);
        camArrow.visible = false;
        fmScene.add(camArrow);
        fmCameraArrows[arm] = camArrow;

        const onDraggingChanged = (e) => {
            fmOrbit.enabled = !e.value;
            if (!e.value) fmOnTargetSettled(arm);
        };
        const onObjectChange = () => fmOnTargetMoved(arm);

        // ONE TransformControls per arm, not two. An earlier version ran a
        // translate-mode instance and a rotate-mode instance side by side, both
        // attached to the same ball, sized so their handles didn't visually overlap
        // -- but a scripted drag test proved that doesn't actually fix it: both
        // instances listen on the same canvas element, and a pure translate-arrow
        // drag ALSO rotated the target (confirmed: position moved exactly as
        // dragged, quaternion changed too, on every run). TransformControls was
        // never designed to be doubled up like that -- rotate mode has a
        // screen-space "free rotate" handle that isn't confined to the visible
        // rings, so it keeps eating drags meant for the other instance regardless of
        // sizing. A single instance with an instant mode switch (G/R, same keys
        // Blender uses, plus the buttons below) has zero ambiguity: whichever mode
        // is active is the only thing that can respond to a drag.
        const gizmo = new THREE.TransformControls(fmCamera, fmRenderer.domElement);
        gizmo.setMode("translate");
        gizmo.setSize(0.8);
        gizmo.attach(target);
        gizmo.visible = false;
        gizmo.enabled = false;
        gizmo.addEventListener("dragging-changed", onDraggingChanged);
        gizmo.addEventListener("objectChange", onObjectChange);
        fmScene.add(gizmo);
        fmGizmos[arm] = gizmo;
    });

    fmLoadUrdf();
    window.addEventListener("resize", fmOnResize);
    fmAnimate();
    fmWireUi();
    fmWireSockets();
}

function fmOnResize() {
    const viewport = document.getElementById("freemove-viewport");
    if (!viewport || !fmRenderer) return;
    fmCamera.aspect = viewport.clientWidth / Math.max(1, viewport.clientHeight);
    fmCamera.updateProjectionMatrix();
    fmRenderer.setSize(viewport.clientWidth, viewport.clientHeight);
}

function fmAnimate() {
    requestAnimationFrame(fmAnimate);
    fmStepJointInterpolation();
    if (fmRenderer) fmRenderer.render(fmScene, fmCamera);
}

function fmStepJointInterpolation() {
    if (!fmRobot) return;
    const alpha = 0.25; // per-frame easing toward the latest target
    let changed = false;
    for (const key in fmJointStateTarget) {
        const target = fmJointStateTarget[key];
        const current = fmJointStateDisplayed[key];
        if (current === undefined) {
            fmJointStateDisplayed[key] = target;
            changed = true;
            continue;
        }
        if (Math.abs(target - current) < 1e-5) continue;
        fmJointStateDisplayed[key] = current + (target - current) * alpha;
        changed = true;
    }
    if (changed) {
        fmRobot.setJointValues(fmJointStateDisplayed);
        fmSyncTrackedSliders();
    }
}

// Keeps each tracked arm's joint sliders (and fmJointGoal, so a stray Plan click uses
// the real pose) glued to the live robot state -- this is what makes the joint panel
// reflect the real robots' current position instead of freezing at whatever it last
// showed, including right after a Cartesian-space Execute (which never touches
// fmJointGoal directly, unlike a joint-space Plan/Execute).
function fmSyncTrackedSliders() {
    Object.keys(FM_ARMS).forEach((arm) => {
        if (!fmJointTracking[arm]) return;
        FM_JOINT_NAMES[arm].forEach((name) => {
            const v = fmJointStateDisplayed[name];
            if (v === undefined) return;
            fmJointGoal[arm][name] = v;
            const slider = document.querySelector(
                `.fm-joint-sliders input[data-arm="${arm}"][data-joint="${name}"]`);
            if (!slider) return;
            slider.value = v;
            const joint = fmRobot && fmRobot.joints[name];
            const isPrismatic = joint && joint.jointType === "prismatic";
            const valueSpan = document.getElementById(`fm-joint-val-${arm}-${name}`);
            if (valueSpan) valueSpan.textContent = fmFormatJointValue(v, isPrismatic);
        });
    });
}

function fmLoadUrdf() {
    const loader = new URDFLoader();
    loader.load(
        "/freemove/urdf",
        (robot) => {
            fmRobot = robot;
            fmScene.add(robot);
            // Snap straight to whatever target state has already arrived (no need to
            // ease in from nothing); fmStepJointInterpolation takes over from here.
            Object.assign(fmJointStateDisplayed, fmJointStateTarget);
            fmRobot.setJointValues(fmJointStateDisplayed);
            fmComputeCameraOffsets();
            fmSeedTargetsFromRobot();
            fmFitCameraToRobot();
            Object.keys(FM_ARMS).forEach(fmBuildJointSliders);
            // Ghosts are loaded FRESH (their own URDFLoader.load() call each), not
            // robot.clone()'d -- measured 2026-09-09: cloning silently dropped the
            // model from 105 meshes down to 8, almost certainly urdf-loader's
            // URDFJoint/URDFLink copy() overrides not deep-cloning children the way
            // a plain Object3D.clone() would. The extra fetches are cheap (every mesh
            // is already in the browser's HTTP cache from loading fmRobot itself, so
            // this is just re-parsing the same bytes, not re-downloading them).
            Object.keys(FM_ARMS).forEach(fmLoadGhost);
        },
        undefined,
        (err) => fmSetBanner("Failed to load the cell URDF: " + err, true),
    );
}

function fmLoadGhost(arm) {
    // Individual mesh files (STL fetches) load ASYNCHRONOUSLY, well after the load()
    // callback below fires -- that callback only receives the parsed link/joint tree,
    // not the actual geometries. Tinting here used to only catch whatever handful of
    // meshes (box/sphere primitives, cache-hit STLs) had already attached by that
    // instant, leaving most of the ghost in its original per-link URDF colors --
    // visually indistinguishable from the real robot. A dedicated LoadingManager's
    // onLoad only fires once EVERY registered load (the URDF itself and every mesh
    // file it references) has actually finished, which is the only reliable point to
    // tint the whole thing.
    const manager = new THREE.LoadingManager();
    const retint = () => {
        const ghost = fmGhosts[arm];
        if (ghost) fmTintGhost(ghost, FM_GHOST_COLOR, FM_ROBOT_LINK_NAMES[arm]);
    };
    manager.onLoad = () => {
        retint();
        // One of the meshes (a caster wheel STL, ~700k vertices) occasionally attaches
        // to the tree a beat after the manager reports every item loaded -- measured via
        // CDP, not theoretical. A cheap idempotent re-sweep catches that straggler
        // without needing to chase down the exact three.js FileLoader dedup race.
        setTimeout(retint, 2500);
    };
    const loader = new URDFLoader(manager);
    loader.load(
        "/freemove/urdf",
        (ghost) => {
            ghost.visible = false;
            fmScene.add(ghost);
            fmGhosts[arm] = ghost;
        },
        undefined,
        (err) => console.error(`[freemove] ghost load failed for ${arm}:`, err),
    );
}

function fmComputeCameraOffsets() {
    if (!fmRobot) return;
    fmRobot.updateMatrixWorld(true);
    Object.keys(FM_ARMS).forEach((arm) => {
        const tcpLink = fmRobot.links[FM_ARMS[arm].tcpLink];
        const camLink = fmRobot.links[FM_CAMERA_LINK[arm]];
        if (!tcpLink || !camLink) {
            console.warn(`[freemove] camera link "${FM_CAMERA_LINK[arm]}" not found for ${arm}`);
            return;
        }
        const tcpInverse = new THREE.Matrix4().copy(tcpLink.matrixWorld).invert();
        const offsetMatrix = new THREE.Matrix4().multiplyMatrices(tcpInverse, camLink.matrixWorld);
        const position = new THREE.Vector3();
        const quaternion = new THREE.Quaternion();
        const scale = new THREE.Vector3();
        offsetMatrix.decompose(position, quaternion, scale);
        fmCameraOffsets[arm] = { position, quaternion };
    });
}

function fmUpdateCameraArrow(arm) {
    const offset = fmCameraOffsets[arm];
    const target = fmTargets[arm];
    const arrow = fmCameraArrows[arm];
    if (!offset || !target || !arrow) return;
    const camPos = offset.position.clone().applyQuaternion(target.quaternion).add(target.position);
    const camQuat = target.quaternion.clone().multiply(offset.quaternion);
    const dir = new THREE.Vector3(0, 0, 1).applyQuaternion(camQuat);
    arrow.position.copy(camPos);
    arrow.setDirection(dir);
}

// --- joint-angle slider panel -------------------------------------------------- //

function fmBuildJointSliders(arm) {
    const container = document.getElementById("fm-joint-sliders-" + arm);
    if (!container || !fmRobot) return;
    container.innerHTML = "";
    fmJointGoal[arm] = {};

    FM_JOINT_NAMES[arm].forEach((name) => {
        const joint = fmRobot.joints[name];
        if (!joint) {
            console.warn(`[freemove] joint "${name}" not found for ${arm}`);
            return;
        }
        const isPrismatic = joint.jointType === "prismatic";
        let lower = joint.limit.lower;
        let upper = joint.limit.upper;
        if (!(upper > lower)) {
            // Degenerate/missing <limit> (e.g. a continuous joint) -- fall back to a
            // full turn so the slider is still usable instead of a zero-width one.
            lower = -Math.PI;
            upper = Math.PI;
        }
        const current = fmJointStateDisplayed[name] !== undefined
            ? fmJointStateDisplayed[name] : (lower + upper) / 2;
        fmJointGoal[arm][name] = current;

        const row = document.createElement("div");
        row.className = "fm-joint-row";
        const labelEl = document.createElement("div");
        labelEl.className = "fm-joint-label";
        const nameSpan = document.createElement("span");
        nameSpan.textContent = FM_JOINT_LABELS[name] || name;
        const valueSpan = document.createElement("span");
        valueSpan.id = `fm-joint-val-${arm}-${name}`;
        valueSpan.textContent = fmFormatJointValue(current, isPrismatic);
        labelEl.appendChild(nameSpan);
        labelEl.appendChild(valueSpan);

        const slider = document.createElement("input");
        slider.type = "range";
        slider.min = lower;
        slider.max = upper;
        slider.step = isPrismatic ? 0.001 : 0.001; // radians/metres internally regardless of display unit
        slider.value = current;
        slider.dataset.arm = arm;
        slider.dataset.joint = name;
        slider.addEventListener("input", () => {
            fmJointTracking[arm] = false; // manual edit -- stop overwriting it live
            const v = parseFloat(slider.value);
            fmJointGoal[arm][name] = v;
            valueSpan.textContent = fmFormatJointValue(v, isPrismatic);
            fmOnJointGoalChanged(arm);
        });

        row.appendChild(labelEl);
        row.appendChild(slider);
        container.appendChild(row);
    });
}

function fmFormatJointValue(radiansOrMetres, isPrismatic) {
    return isPrismatic
        ? `${radiansOrMetres.toFixed(3)} m`
        : `${(radiansOrMetres * 180 / Math.PI).toFixed(1)}°`;
}

function fmOnJointGoalChanged(arm) {
    const ghost = fmGhosts[arm];
    if (ghost) {
        ghost.setJointValues(fmJointGoal[arm]);
        ghost.visible = true;
    }
    fmJointPlanValidFor[arm] = null;
    const execBtn = document.querySelector(`.fm-execute-joint-btn[data-arm="${arm}"]`);
    if (execBtn) execBtn.disabled = true;
}

function fmSyncJointsToCurrent(arm) {
    fmJointTracking[arm] = true;
    FM_JOINT_NAMES[arm].forEach((name) => {
        if (fmJointStateDisplayed[name] === undefined) return;
        fmJointGoal[arm][name] = fmJointStateDisplayed[name];
        const slider = document.querySelector(
            `.fm-joint-sliders input[data-arm="${arm}"][data-joint="${name}"]`);
        if (slider) {
            slider.value = fmJointStateDisplayed[name];
            const joint = fmRobot && fmRobot.joints[name];
            const isPrismatic = joint && joint.jointType === "prismatic";
            const valueSpan = document.getElementById(`fm-joint-val-${arm}-${name}`);
            if (valueSpan) valueSpan.textContent = fmFormatJointValue(fmJointStateDisplayed[name], isPrismatic);
        }
    });
    fmOnJointGoalChanged(arm);
}

function fmFitCameraToRobot() {
    if (!fmRobot) return;
    fmRobot.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(fmRobot);
    if (box.isEmpty()) return;

    const size = new THREE.Vector3();
    const center = new THREE.Vector3();
    box.getSize(size);
    box.getCenter(center);
    const radius = Math.max(size.x, size.y, size.z, 1) * 0.65;

    fmOrbit.target.copy(center);
    // A steeper, more top-down-ish default (measured 2026-09-09, not the more
    // "classic" 45/45/35 isometric-looking (1.4,-1.4,1.1) this used to be): from that
    // angle the Kawasaki gizmo's X and Y translate arrows projected only ~28 deg apart
    // on screen and both were badly foreshortened, at this cell's actual layout and
    // Kawasaki's screen position -- easy to visually mistake one for the other, which
    // reads exactly like "dragging X moves Y" even though the underlying axes are
    // correct. (0.9,-0.9,1.9) keeps the SAME camera distance from center but widens
    // that on-screen angle to ~77 deg for Kawasaki (and ~87 deg for the UR, which was
    // already fine), verified by projecting both arms' axis tips to screen space.
    fmCamera.position.set(
        center.x + radius * 0.9,
        center.y - radius * 0.9,
        center.z + radius * 1.9,
    );
    fmCamera.near = Math.max(0.01, radius / 200);
    fmCamera.far = radius * 30;
    fmCamera.updateProjectionMatrix();
    fmOrbit.update();

    if (fmGrid) {
        fmScene.remove(fmGrid);
        fmGrid.geometry.dispose();
        fmGrid.material.dispose();
    }
    const gridSize = Math.max(radius * 4, 4);
    fmGrid = new THREE.GridHelper(gridSize, 40, 0x334155, 0x1e293b);
    fmGrid.rotation.x = Math.PI / 2;
    fmGrid.position.set(center.x, center.y, box.min.z);
    fmScene.add(fmGrid);
}

function fmTintGhost(root, color, allowedLinkNames) {
    root.traverse((child) => {
        if (!child.isMesh || !child.material) return;
        let n = child, linkName = null;
        while (n) {
            if (n.isURDFLink) { linkName = n.name; break; }
            n = n.parent;
        }
        if (!allowedLinkNames || !allowedLinkNames.has(linkName)) {
            // Environmental part (table, chassis, AGV base/wheels, casters, ...) --
            // it's static and identical to the real robot's own copy at the exact same
            // pose, so showing it again here would only ever be a redundant overlay.
            child.visible = false;
            return;
        }
        const mat = child.material.clone();
        mat.transparent = true;
        // Prominent, not a faint outline: this is the operator's one visual cue
        // for "where will the arm/camera end up", so it needs to read clearly
        // against the solid live robot, not just tint it slightly.
        mat.opacity = 0.6;
        mat.emissive = new THREE.Color(color);
        mat.emissiveIntensity = 0.5;
        mat.depthWrite = false;
        mat.color = new THREE.Color(color);
        child.material = mat;
        child.renderOrder = 1;
        child.visible = true;
    });
}

function fmSeedTargetFromRobot(arm) {
    if (!fmRobot) return;
    fmRobot.updateMatrixWorld(true);
    const link = fmRobot.links[FM_ARMS[arm].tcpLink];
    const target = fmTargets[arm];
    if (!link || !target) return;
    // robot and target are both direct children of `scene` with nothing rotated
    // anywhere, so the link's world pose IS the target's local pose -- no conversion.
    link.getWorldPosition(target.position);
    link.getWorldQuaternion(target.quaternion);
    fmUpdatePoseReadout(arm);
    fmUpdateCameraArrow(arm);
    fmInvalidatePlan(arm);
}

function fmSeedTargetsFromRobot() {
    Object.keys(FM_ARMS).forEach(fmSeedTargetFromRobot);
}

// --- live robot pose from the existing joint_states broadcast --------------- //
//
// /joint_states on the real cell is split across several publishers that each only
// cover part of the robot (see the matching note in free_move.py's module docstring
// and project memory anomali-joint-states-cok-yayinci) -- a single message is never
// "the whole state". fmJointStateTarget accumulates every name seen, by name, across
// every message, so the model always reflects the union of the latest values rather
// than whichever publisher happened to fire last -- it converges to the true
// full state within a couple of pushes even though any single message only covers
// part of the robot. fmJointStateDisplayed is what's actually applied to fmRobot;
// the animate loop eases it toward the target every frame (see fmAnimate) instead of
// snapping fmRobot straight to fmJointStateTarget on each ~5Hz socket push, which is
// what made the live robot look stepped/jerky -- 60fps rendering with the pose only
// changing 5x/second reads as stutter no matter how fast the render loop itself is.
const fmJointStateTarget = {};
const fmJointStateDisplayed = {};

if (typeof socket !== "undefined") {
    socket.on("joint_states", (data) => {
        const series = (data && data.real) || [];
        if (!series.length) return;
        const latest = series[series.length - 1];
        for (const key in latest) {
            if (key === "t") continue;
            fmJointStateTarget[key] = latest[key];
        }
    });
}

// --- gizmo drag handling ----------------------------------------------------- //

function fmOnTargetMoved(arm) {
    // No IK check here on purpose. This fires on every objectChange event while
    // actively dragging (tens of times per second) -- hitting /compute_ik that
    // often was loading move_group enough that it eventually started timing out
    // sessionwide, not just for the drag itself. The camera-direction arrow still
    // updates live (pure local math, no round trip); the ghost/reachability check
    // now only runs once the ball is actually released, or on Plan.
    fmUpdatePoseReadout(arm);
    fmUpdateCameraArrow(arm);
    fmInvalidatePlan(arm);
}

function fmOnTargetSettled(arm) {
    fmRequestCheck(arm);
}

function fmRequestCheck(arm) {
    if (!fmActive || !fmArmReady[arm]) return;
    const pose = fmReadTargetPose(arm);
    if (!pose) return;
    socket.emit("freemove_check", { arm, position: pose.position, quat_xyzw: pose.quat_xyzw });
}

function fmReadTargetPose(arm) {
    const target = fmTargets[arm];
    if (!target) return null;
    return {
        position: [target.position.x, target.position.y, target.position.z],
        quat_xyzw: [target.quaternion.x, target.quaternion.y, target.quaternion.z, target.quaternion.w],
    };
}

function fmUpdatePoseReadout(arm) {
    const el = document.getElementById("fm-pose-" + arm);
    const pose = fmReadTargetPose(arm);
    if (!el || !pose) return;
    const [x, y, z] = pose.position;
    el.textContent =
        `x ${x.toFixed(3)}  y ${y.toFixed(3)}  z ${z.toFixed(3)}`;
}

function fmInvalidatePlan(arm) {
    // Deliberately does NOT touch ghost visibility. This runs on every single
    // objectChange event while dragging (many per second) -- hiding the ghost here
    // meant it was blanked on every frame and only ever flashed on for the instant
    // between an IK response landing and the next mousemove, i.e. never visibly on
    // during an actual drag. The ghost is now owned entirely by the IK-result handler:
    // it keeps showing the last good preview until a fresh result says otherwise.
    fmPlanValidFor[arm] = null;
    const execBtn = document.querySelector(`.fm-execute-btn[data-arm="${arm}"]`);
    if (execBtn) execBtn.disabled = true;
    fmClearCameraTrail(arm);
}

function fmClearCameraTrail(arm) {
    const line = fmCameraTrails[arm];
    if (!line) return;
    fmScene.remove(line);
    line.geometry.dispose();
    line.material.dispose();
    fmCameraTrails[arm] = null;
}

function fmPoseKey(pose) {
    return JSON.stringify([
        pose.position.map((v) => v.toFixed(4)),
        pose.quat_xyzw.map((v) => v.toFixed(4)),
    ]);
}

// --- UI wiring ---------------------------------------------------------------- //

function fmWireUi() {
    document.getElementById("freemove-enable-btn").addEventListener("click", () => {
        if (fmActive) {
            if (confirm("Disable Free Move and shut down HIL for both arms?")) {
                socket.emit("freemove_stop");
            }
            return;
        }
        const useFakeHardware = document.getElementById("freemove-fake-hw-toggle").checked;
        socket.emit("freemove_start", { use_fake_hardware: useFakeHardware });
        fmSetBanner("Starting HIL and bringing up both arms — this can take up to a minute...", false);
    });

    document.getElementById("freemove-stop-btn").addEventListener("click", () => {
        if (typeof emergencyStop === "function") emergencyStop();
    });

    document.querySelectorAll(".fm-mode-btn").forEach((btn) => {
        btn.addEventListener("click", () => fmSetGizmoMode(btn.dataset.arm, btn.dataset.mode));
    });

    // G/R switch mode instantly without reaching for a button, same keys Blender
    // uses. Applies to whichever arm the pointer is over so both balls can be
    // driven without re-clicking a button for every switch; falls back to "ur" if
    // the pointer hasn't been over either card yet. Ignored while typing into a
    // settings field, and while the tab isn't visible.
    let fmHoveredArm = "ur";
    document.querySelectorAll(".fm-arm-card").forEach((card) => {
        card.addEventListener("pointerenter", () => { fmHoveredArm = card.dataset.arm; });
    });
    window.addEventListener("keydown", (e) => {
        const view = document.getElementById("view-freemove");
        if (!view || view.hidden) return;
        const tag = (document.activeElement && document.activeElement.tagName) || "";
        if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
        if (e.key === "g" || e.key === "G") fmSetGizmoMode(fmHoveredArm, "translate");
        else if (e.key === "r" || e.key === "R") fmSetGizmoMode(fmHoveredArm, "rotate");
    });

    document.querySelectorAll(".fm-plan-btn").forEach((btn) => {
        btn.addEventListener("click", () => {
            const arm = btn.dataset.arm;
            const pose = fmReadTargetPose(arm);
            if (!pose) return;
            fmSetArmStatus(arm, "Planning...", "busy");
            socket.emit("freemove_plan", { arm, position: pose.position, quat_xyzw: pose.quat_xyzw });
        });
    });

    document.querySelectorAll(".fm-execute-btn").forEach((btn) => {
        btn.addEventListener("click", () => {
            const arm = btn.dataset.arm;
            const pose = fmReadTargetPose(arm);
            if (!pose) return;
            if (fmPlanValidFor[arm] !== fmPoseKey(pose)) {
                alert("Plan this pose again before executing — it changed since the last Plan.");
                return;
            }
            if (!confirm(`Move the real ${FM_ARMS[arm].label} to this pose now?`)) return;
            socket.emit("freemove_execute", { arm, position: pose.position, quat_xyzw: pose.quat_xyzw });
        });
    });

    document.querySelectorAll(".fm-cancel-btn").forEach((btn) => {
        btn.addEventListener("click", () => {
            socket.emit("freemove_cancel", { arm: btn.dataset.arm });
        });
    });

    document.querySelectorAll(".fm-reset-btn").forEach((btn) => {
        btn.addEventListener("click", () => fmSeedTargetFromRobot(btn.dataset.arm));
    });

    document.querySelectorAll(".fm-sync-joints-btn").forEach((btn) => {
        btn.addEventListener("click", () => fmSyncJointsToCurrent(btn.dataset.arm));
    });

    document.querySelectorAll(".fm-plan-joint-btn").forEach((btn) => {
        btn.addEventListener("click", () => {
            const arm = btn.dataset.arm;
            fmSetArmStatus(arm, "Planning (joints)...", "busy");
            socket.emit("freemove_plan_joint", { arm, joint_positions: fmJointGoal[arm] });
        });
    });

    document.querySelectorAll(".fm-execute-joint-btn").forEach((btn) => {
        btn.addEventListener("click", () => {
            const arm = btn.dataset.arm;
            const key = JSON.stringify(fmJointGoal[arm]);
            if (fmJointPlanValidFor[arm] !== key) {
                alert("Plan these joint angles again before executing — they changed since the last Plan.");
                return;
            }
            if (!confirm(`Move the real ${FM_ARMS[arm].label} to these joint angles now?`)) return;
            socket.emit("freemove_execute_joint", { arm, joint_positions: fmJointGoal[arm] });
        });
    });

    document.querySelectorAll(".fm-apply-settings-btn").forEach((btn) => {
        btn.addEventListener("click", () => {
            const arm = btn.dataset.arm;
            const params = {};
            document.querySelectorAll(`.fm-param-input[data-arm="${arm}"][data-key]`).forEach((input) => {
                const key = input.dataset.key;
                params[key] = key === "planner_id" ? input.value
                    : key === "num_planning_attempts" ? parseInt(input.value, 10)
                        : parseFloat(input.value);
            });
            socket.emit("freemove_set_params", { arm, params });
        });
    });

    document.querySelectorAll(".fm-apply-padding-btn").forEach((btn) => {
        btn.addEventListener("click", () => {
            const arm = btn.dataset.arm;
            const input = document.getElementById("fm-padding-" + arm);
            if (!input) return;
            socket.emit("freemove_set_padding", { arm, padding_m: parseFloat(input.value) });
        });
    });

    const logContainer = document.getElementById("freemove-log-container");
    document.getElementById("freemove-log-autoscroll").addEventListener("click", () => {
        fmLogAutoScroll = !fmLogAutoScroll;
        document.getElementById("freemove-autoscroll-label").textContent = fmLogAutoScroll ? "ON" : "OFF";
    });
    document.getElementById("freemove-log-clear").addEventListener("click", () => {
        if (logContainer) logContainer.innerHTML = "";
    });
}

function fmSetBanner(text, isError) {
    const banner = document.getElementById("freemove-banner");
    if (!banner) return;
    if (!text) {
        banner.hidden = true;
        return;
    }
    banner.hidden = false;
    banner.textContent = text;
    banner.style.borderColor = isError ? "var(--accent-red)" : "var(--accent-yellow)";
    banner.style.color = isError ? "var(--accent-red)" : "var(--accent-yellow)";
    banner.style.background = isError ? "rgba(239, 68, 68, 0.1)" : "rgba(245, 158, 11, 0.1)";
}

function fmSetGizmoMode(arm, mode) {
    fmGizmoMode[arm] = mode;
    if (fmGizmos[arm]) fmGizmos[arm].setMode(mode);
    document.querySelectorAll(`.fm-mode-btn[data-arm="${arm}"]`).forEach((btn) => {
        btn.classList.toggle("active", btn.dataset.mode === mode);
    });
}

function fmSetArmStatus(arm, text, cls) {
    const el = document.getElementById("fm-status-" + arm);
    if (!el) return;
    el.textContent = text;
    el.className = "fm-status" + (cls ? " " + cls : "");
}

function fmSetEnabledUi(active) {
    fmActive = active;
    const enableBtn = document.getElementById("freemove-enable-btn");
    if (enableBtn) enableBtn.textContent = active ? "Disable Free Move" : "Enable Free Move";

    Object.keys(FM_ARMS).forEach((arm) => {
        const ready = active && fmArmReady[arm];
        if (fmGizmos[arm]) {
            fmGizmos[arm].visible = ready;
            fmGizmos[arm].enabled = ready;
        }
        if (fmTargets[arm]) fmTargets[arm].visible = ready;
        if (fmCameraArrows[arm]) {
            fmCameraArrows[arm].visible = ready;
            if (ready) fmUpdateCameraArrow(arm);
        }
        if (!ready) fmClearCameraTrail(arm);
        document.querySelectorAll(
            `.fm-plan-btn[data-arm="${arm}"], .fm-reset-btn[data-arm="${arm}"], `
            + `.fm-plan-joint-btn[data-arm="${arm}"], .fm-sync-joints-btn[data-arm="${arm}"]`)
            .forEach((btn) => { btn.disabled = !ready; });
        // Both Execute buttons stay disabled until a fresh successful Plan, regardless.
        if (!ready) fmSetArmStatus(arm, active ? "Waiting for arm..." : "Not enabled.", "");
    });
}

// --- socket wiring ------------------------------------------------------------ //

function fmWireSockets() {
    socket.on("status_update", (data) => {
        if (!data) return;
        fmSetEnabledUi(!!data.free_move_active);
        if (data.free_move_active) {
            fmSetBanner(null, false);
        } else if (data.hil_status !== "stopped") {
            fmSetBanner("A scenario is using the cell. Stop it from the Dashboard tab to use Free Move.", false);
        } else {
            fmSetBanner(null, false);
        }
    });

    socket.on("freemove_ready", (data) => {
        ["ur", "kawasaki"].forEach((arm) => {
            const info = data && data[arm];
            fmArmReady[arm] = !!(info && info.ready);
            fmSetEnabledUi(fmActive);
            if (fmArmReady[arm]) {
                fmSetArmStatus(arm, "Ready.", "ok");
                if (info.pose && fmTargets[arm]) {
                    fmTargets[arm].position.fromArray(info.pose.position);
                    fmTargets[arm].quaternion.fromArray(info.pose.quat_xyzw);
                    fmUpdatePoseReadout(arm);
                }
                if (info.params) fmApplyParamsToInputs(arm, info.params);
                fmPopulatePlanners(arm, info.planner_ids || []);
            } else {
                fmSetArmStatus(arm, "Failed to initialize — check the log.", "bad");
            }
        });
    });

    socket.on("freemove_params_result", (data) => {
        if (data && data.ok && data.params) fmApplyParamsToInputs(data.arm, data.params);
    });

    // freemove_padding_result: no extra UI beyond the shared log terminal below,
    // which already gets a log_message for every padding attempt from the backend.

    socket.on("log_message", (data) => {
        const container = document.getElementById("freemove-log-container");
        if (!container || !data) return;
        const entry = document.createElement("div");
        entry.className = "log-entry";
        const src = (data.source || "").toLowerCase();
        let sourceClass = "system";
        if (src.includes("hil")) sourceClass = "hil";
        else if (src.includes("senaryo") || src.includes("scenario") || src.includes("freemove"))
            sourceClass = "scenario";
        const timeEl = document.createElement("span");
        timeEl.className = "log-time";
        timeEl.textContent = data.timestamp || "";
        const sourceEl = document.createElement("span");
        sourceEl.className = "log-source " + sourceClass;
        sourceEl.textContent = data.source || "";
        const msgEl = document.createElement("span");
        msgEl.className = "log-msg";
        msgEl.textContent = data.message || "";
        entry.appendChild(timeEl);
        entry.appendChild(sourceEl);
        entry.appendChild(msgEl);
        container.appendChild(entry);
        while (container.children.length > FM_MAX_LOG_ENTRIES) {
            container.removeChild(container.firstChild);
        }
        if (fmLogAutoScroll) container.scrollTop = container.scrollHeight;
    });

    socket.on("freemove_ik_result", (data) => {
        if (!data) return;
        const { arm, ok, joint_positions, collision_free, error } = data;
        const target = fmTargets[arm];
        if (target) {
            // Red: no IK solution at all (unreachable). Orange: reachable but the pose
            // collides with something. Green: reachable and clear. The ghost is shown
            // for BOTH orange and green -- collision-aware IK is deliberately off for
            // this live preview (see check_ik()'s docstring), so the operator always
            // sees where the arm/camera would end up, the same way RViz's own goal
            // marker does, instead of the ball just going dark with no explanation.
            let color = 0xef4444;
            if (ok) color = collision_free === false ? 0xf59e0b : 0x22c55e;
            target.material.color.set(color);
            if (fmCameraArrows[arm]) fmCameraArrows[arm].setColor(color);
        }
        const ghost = fmGhosts[arm];
        if (ghost && ok && joint_positions) {
            ghost.setJointValues(joint_positions);
            ghost.visible = true;
            fmSetArmStatus(arm,
                collision_free === false ? "Reachable, but colliding." : "Reachable.",
                collision_free === false ? "busy" : "ok");
        } else if (ghost) {
            ghost.visible = false;
            fmSetArmStatus(arm, "Unreachable" + (error ? " (" + error + ")" : "") + ".", "bad");
        }
    });

    socket.on("freemove_plan_result", (data) => {
        if (!data) return;
        const { arm, ok, trajectory, error } = data;
        const execBtn = document.querySelector(`.fm-execute-btn[data-arm="${arm}"]`);
        if (!ok) {
            fmSetArmStatus(arm, "Plan failed: " + (error || "unknown error"), "bad");
            if (execBtn) execBtn.disabled = true;
            fmPlanValidFor[arm] = null;
            return;
        }
        const pose = fmReadTargetPose(arm);
        fmPlanValidFor[arm] = pose ? fmPoseKey(pose) : null;
        if (execBtn) execBtn.disabled = false;
        fmSetArmStatus(arm, "Plan OK — ready to execute.", "ok");
        fmLastTrajectory[arm] = trajectory;
        fmAnimatePlan(arm, trajectory);
    });

    socket.on("freemove_plan_joint_result", (data) => {
        if (!data) return;
        const { arm, ok, trajectory, error } = data;
        const execBtn = document.querySelector(`.fm-execute-joint-btn[data-arm="${arm}"]`);
        if (!ok) {
            fmSetArmStatus(arm, "Joint plan failed: " + (error || "unknown error"), "bad");
            if (execBtn) execBtn.disabled = true;
            fmJointPlanValidFor[arm] = null;
            return;
        }
        fmJointPlanValidFor[arm] = JSON.stringify(fmJointGoal[arm]);
        if (execBtn) execBtn.disabled = false;
        fmSetArmStatus(arm, "Joint plan OK — ready to execute.", "ok");
        fmLastTrajectory[arm] = trajectory;
        fmAnimatePlan(arm, trajectory);
    });

    socket.on("freemove_execute_status", (data) => {
        if (!data) return;
        const { arm, state, error } = data;
        if (state === "executing") {
            fmSetArmStatus(arm, "Executing...", "busy");
            // Re-play the exact same trajectory, from the top, timed to the real
            // waypoint deltas -- this is what keeps the ghost showing the real robot's
            // ACTUAL planned path while it moves, instead of sitting frozen at the
            // Plan-preview's final pose while the real (possibly much slower -- e.g.
            // Kawasaki's default velocity scale is 0.02) move is still underway.
            if (fmLastTrajectory[arm]) fmAnimatePlan(arm, fmLastTrajectory[arm]);
        } else if (state === "succeeded") {
            fmSetArmStatus(arm, "Move complete.", "ok");
            // Either goal domain could have just executed; the robot's start state
            // changed either way, so both kinds of pending plan are now stale.
            fmInvalidatePlan(arm);
            fmJointPlanValidFor[arm] = null;
            const jointExecBtn = document.querySelector(`.fm-execute-joint-btn[data-arm="${arm}"]`);
            if (jointExecBtn) jointExecBtn.disabled = true;
            // The robot just moved (Cartesian OR joint-space) -- resume live tracking
            // so the joint panel reflects where it actually ended up, not the
            // pre-move pose the sliders were last showing.
            fmJointTracking[arm] = true;
            fmSyncTrackedSliders();
        } else {
            fmSetArmStatus(arm, "Move failed: " + (error || "unknown error"), "bad");
        }
    });
}

function fmPopulatePlanners(arm, plannerIds) {
    const select = document.getElementById("fm-planner-" + arm);
    if (!select) return;
    const current = select.value;
    select.innerHTML = '<option value="">Auto (RRTConnect)</option>';
    plannerIds.forEach((pid) => {
        const opt = document.createElement("option");
        opt.value = pid;
        opt.textContent = pid;
        select.appendChild(opt);
    });
    // Keep whatever was selected if it's still a valid option (e.g. re-enabling
    // Free Move shouldn't silently reset an operator's planner choice).
    if ([...select.options].some((o) => o.value === current)) select.value = current;
}

function fmApplyParamsToInputs(arm, params) {
    document.querySelectorAll(`.fm-param-input[data-arm="${arm}"][data-key]`).forEach((input) => {
        const value = params[input.dataset.key];
        if (value !== undefined && value !== null) input.value = value;
    });
    if (params.padding_m !== undefined) {
        const padInput = document.getElementById("fm-padding-" + arm);
        if (padInput) padInput.value = params.padding_m;
    }
}

function fmSetCameraTrail(arm, points) {
    fmClearCameraTrail(arm);
    if (!points || points.length < 2) return;
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineBasicMaterial({ color: FM_ARMS[arm].color });
    const line = new THREE.Line(geometry, material);
    fmScene.add(line);
    fmCameraTrails[arm] = line;
}

function fmAnimatePlan(arm, trajectory) {
    const ghost = fmGhosts[arm];
    if (!ghost || !trajectory || !trajectory.points || !trajectory.points.length) return;
    ghost.visible = true;
    const names = trajectory.joint_names;
    const points = trajectory.points;

    // Walk every waypoint once up front to trace the camera's actual path through
    // space (every joint between the TCP and the SICK frame is fixed, but the arm
    // joints in between are not, so this DOES need real FK per waypoint, unlike the
    // live drag preview which can use the constant TCP->camera offset directly).
    const camLink = ghost.links[FM_CAMERA_LINK[arm]];
    if (camLink) {
        const trail = [];
        points.forEach((pt) => {
            const values = {};
            names.forEach((n, idx) => { values[n] = pt.positions[idx]; });
            ghost.setJointValues(values);
            ghost.updateMatrixWorld(true);
            trail.push(camLink.getWorldPosition(new THREE.Vector3()));
        });
        fmSetCameraTrail(arm, trail);
    }

    // Paced by each waypoint's real time_from_start delta, not a fixed interval -- so
    // this actually takes as long as the planned trajectory really will (including
    // this arm's velocity/acceleration scale), whether it's playing as the Plan
    // preview or being re-called to stay in sync with a real Execute.
    const myGen = ++fmAnimGeneration[arm];
    const applyWaypoint = (idx) => {
        const values = {};
        names.forEach((n, j) => { values[n] = points[idx].positions[j]; });
        ghost.setJointValues(values);
    };
    const step = (idx) => {
        if (fmAnimGeneration[arm] !== myGen) return; // superseded by a newer Plan/Execute
        applyWaypoint(idx);
        if (idx + 1 >= points.length) return;
        const dtSec = (points[idx + 1].time_from_start || 0) - (points[idx].time_from_start || 0);
        setTimeout(() => step(idx + 1), Math.max(16, dtSec * 1000));
    };
    step(0);
}
