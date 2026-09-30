#!/usr/bin/env python3
"""Static check of an STLC-generated test file against robot_capabilities.json.

Rejects code that imports or calls things that do not exist in the harness image,
uses out-of-range scaling factors or joint targets, mocks the robot, or never ends.
It only reads the file (AST), so it is safe to run before anything is sent to IFARLAB.

    python3 validate_generated.py generated.py [--caps robot_capabilities.json] [--json]

Exit code: 0 = no errors, 1 = errors found, 2 = file does not parse.
"""

import argparse
import ast
import json
import re
import sys
from pathlib import Path

CONTROLLER_CLASSES = {"SimCollisionAwareRobotController", "CollisionAwareRobotController"}
STD_OK = {"rclpy", "pytest", "time", "math"}


def load_caps(path):
    caps = json.loads(Path(path).read_text())
    api = caps["api"]
    ctrl_members = set()
    for cls in api["classes"].values():
        ctrl_members |= set(cls["methods"])
        ctrl_members |= set(cls.get("public_attributes", {}))
    ctrl_members |= {"moveit2", "get_logger", "destroy_node"}
    moveit_members = {re.match(r"\w+", k).group(0) for k in api["moveit2_interface"]["allowed_members"]}
    forbidden_moveit = {re.match(r"\w+", k).group(0) for k in api["forbidden"]["exists_but_do_not_use"]
                        if k.startswith("moveit2.")} | {"compute_fk", "compute_ik", "compute_fk_async",
                                                        "compute_ik_async", "execute", "plan_async", "reset_controller",
                                                        "clear_all_collision_objects", "motion_suceeded"}
    forbidden_moveit.discard("moveit2")
    allowed_from = {}
    for line in api["allowed_imports"]:
        m = re.match(r"from ([\w.]+) import (\w+)", line)
        if m:
            allowed_from.setdefault(m.group(1), set()).add(m.group(2))
    joints = caps["limits"]["joints"]["items"]
    return caps, ctrl_members, moveit_members, forbidden_moveit, allowed_from, joints


class Checker(ast.NodeVisitor):
    def __init__(self, caps_bundle):
        (self.caps, self.ctrl_members, self.moveit_members,
         self.forbidden_moveit, self.allowed_from, self.joints) = caps_bundle
        self.errors = []
        self.warnings = []
        self.ctrl_names = set()  # variables / fixture names that hold a controller

    def err(self, node, msg):
        self.errors.append({"line": getattr(node, "lineno", 0), "msg": msg})

    def warn(self, node, msg):
        self.warnings.append({"line": getattr(node, "lineno", 0), "msg": msg})

    # --- pass 1: find which names hold a controller instance -----------------
    def collect_controller_names(self, tree):
        for node in ast.walk(tree):
            if isinstance(node, (ast.Assign, ast.AnnAssign)) and isinstance(node.value, ast.Call):
                if _call_name(node.value) in CONTROLLER_CLASSES:
                    targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                    for t in targets:
                        if isinstance(t, ast.Name):
                            self.ctrl_names.add(t.id)
                        elif isinstance(t, ast.Attribute):
                            self.ctrl_names.add(t.attr)  # self.robot = Ctrl()
            if isinstance(node, ast.FunctionDef):
                # pytest fixture that returns/yields a controller -> its name is a controller
                for sub in ast.walk(node):
                    if isinstance(sub, (ast.Return, ast.Yield)) and sub.value is not None:
                        v = sub.value
                        if (isinstance(v, ast.Call) and _call_name(v) in CONTROLLER_CLASSES) or \
                                (isinstance(v, ast.Name) and v.id in self.ctrl_names):
                            self.ctrl_names.add(node.name)

    # --- imports --------------------------------------------------------------
    def visit_Import(self, node):
        for a in node.names:
            root = a.name.split(".")[0]
            if root not in STD_OK:
                self.err(node, f"import {a.name}: not in allowed_imports")
        self.generic_visit(node)

    def visit_ImportFrom(self, node):
        mod = node.module or ""
        for a in node.names:
            if mod in self.allowed_from:
                if a.name not in self.allowed_from[mod]:
                    self.err(node, f"from {mod} import {a.name}: '{a.name}' does not exist / is not allowed")
            elif mod.split(".")[0] in STD_OK:
                continue
            elif mod.startswith("unittest.mock") or mod == "unittest":
                self.err(node, f"from {mod} import {a.name}: mocking the robot is not allowed")
            else:
                self.err(node, f"from {mod} import {a.name}: module not in allowed_imports")
        self.generic_visit(node)

    # --- attribute access -----------------------------------------------------
    def visit_Attribute(self, node):
        v = node.value
        # <controller>.moveit2.<member>
        if isinstance(v, ast.Attribute) and v.attr == "moveit2":
            if node.attr in self.forbidden_moveit:
                self.err(node, f"moveit2.{node.attr}: exists but is forbidden (see api.forbidden)")
            elif node.attr not in self.moveit_members:
                self.err(node, f"moveit2.{node.attr}: not an allowed MoveIt2 member")
        # <controller>.<member>
        base = v.id if isinstance(v, ast.Name) else (v.attr if isinstance(v, ast.Attribute) else None)
        if base in self.ctrl_names and node.attr not in self.ctrl_members:
            self.err(node, f"{base}.{node.attr}: controller has no such method/attribute")
        self.generic_visit(node)

    # --- assignments to scaling factors ---------------------------------------
    def visit_Assign(self, node):
        for t in node.targets:
            if isinstance(t, ast.Attribute) and t.attr in ("max_velocity", "max_acceleration"):
                val = _num(node.value)
                if val is not None and not (0.0 < val <= 1.0):
                    self.err(node, f"{t.attr} = {val}: scaling factor must be in (0, 1]")
                elif val is not None and val > self.caps["limits"]["scaling"]["recommended_max_for_tests"]:
                    self.warn(node, f"{t.attr} = {val}: above recommended_max_for_tests")
        self.generic_visit(node)

    # --- calls ----------------------------------------------------------------
    def visit_Call(self, node):
        name = _call_name(node)
        if name in ("move_to_joint_angles", "move_to_configuration") and node.args:
            vec = _num_list(node.args[0])
            if vec is not None:
                if len(vec) != len(self.joints):
                    self.err(node, f"{name}: {len(vec)} values given, need {len(self.joints)} "
                                   "(rail in m + 6 joints in rad)")
                else:
                    for j, x in zip(self.joints, vec):
                        if not (j["position_min"] <= x <= j["position_max"]):
                            self.warn(node, f"{name}: {j['name']}={x} outside "
                                            f"[{j['position_min']}, {j['position_max']}] - only valid as a negative test")
        if name in ("patch", "MagicMock", "Mock"):
            self.err(node, f"{name}(): mocking the robot is not allowed")
        if name == "main" and isinstance(node.func, ast.Attribute) and _call_name_base(node.func) == "pytest":
            for s in ast.walk(node):
                if isinstance(s, ast.Constant) and isinstance(s.value, str) and s.value.startswith("--html"):
                    self.err(node, "pytest --html: pytest-html is not installed in the image")
        if name == "sleep":
            val = _num(node.args[0]) if node.args else None
            if val is not None and val > 60:
                self.warn(node, f"time.sleep({val}): long sleep eats into the 300 s test timeout")
        self.generic_visit(node)

    def visit_While(self, node):
        t = node.test
        endless = (isinstance(t, ast.Constant) and t.value is True) or \
                  (isinstance(t, ast.Call) and _call_name(t) == "ok")
        if endless and not any(isinstance(n, (ast.Break, ast.Return)) for n in ast.walk(node)):
            self.err(node, "endless loop: every run must terminate")
        self.generic_visit(node)


def _call_name(call):
    f = call.func
    return f.id if isinstance(f, ast.Name) else (f.attr if isinstance(f, ast.Attribute) else None)


def _call_name_base(attr):
    return attr.value.id if isinstance(attr.value, ast.Name) else None


def _num(node):
    try:
        v = ast.literal_eval(node)
        return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else None
    except Exception:
        return None


def _num_list(node):
    if not isinstance(node, (ast.List, ast.Tuple)):
        return None
    vals = [_num(e) for e in node.elts]
    return None if any(v is None for v in vals) else vals


def validate(source, caps_path):
    tree = ast.parse(source)
    c = Checker(load_caps(caps_path))
    c.collect_controller_names(tree)
    c.visit(tree)
    if not any(isinstance(n, ast.FunctionDef) and n.name.startswith("test_") for n in ast.walk(tree)):
        c.warn(tree, "no test_ functions: file will run as a plain script (no timeout)")
    return c.errors, c.warnings


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--caps", default=str(Path(__file__).with_name("robot_capabilities.json")))
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    try:
        errors, warnings = validate(Path(a.file).read_text(errors="replace"), a.caps)
    except SyntaxError as e:
        print(f"SYNTAX ERROR line {e.lineno}: {e.msg}")
        return 2
    if a.json:
        print(json.dumps({"errors": errors, "warnings": warnings}, ensure_ascii=False, indent=1))
    else:
        for e in errors:
            print(f"ERROR   line {e['line']}: {e['msg']}")
        for w in warnings:
            print(f"WARNING line {w['line']}: {w['msg']}")
        print(f"{len(errors)} error(s), {len(warnings)} warning(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
