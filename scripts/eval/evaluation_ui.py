#!/usr/bin/env python3
"""Desktop operator UI for the frozen real-robot evaluation runner.

The GUI deliberately delegates robot control, logging, validation, and retry
rules to ``run_evaluation.sh``.  It only renders progress and sends the exact
interactive answers that the validated terminal runner accepts.
"""

from __future__ import annotations

import codecs
from dataclasses import dataclass
import os
from pathlib import Path
import pty
import queue
import re
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER = REPO_ROOT / "scripts/eval/run_evaluation.sh"
OPEN_GRIPPER = REPO_ROOT / "scripts/robot/open_gripper.sh"
MAX_CONSOLE_LINES = 5_000
POLICY_TIMEOUT_S = 60.0

POSITION_COORDS = {
    "G1": (55, 45), "G2": (195, 45), "G3": (335, 45),
    "G4": (55, 155), "G5": (195, 155), "G6": (335, 155),
    "G7": (55, 265), "G8": (195, 265), "G9": (335, 265),
    "Q1": (125, 100), "Q2": (265, 100),
    "Q3": (125, 210), "Q4": (265, 210),
}

PROMPT_PATTERNS = {
    "ready": re.compile(r"Type READY:\s*$"),
    "home": re.compile(r"Type HOME \(the robot will move\):\s*$"),
    "open": re.compile(r"type OPEN:\s*$"),
    "placed": re.compile(r"type PLACED:\s*$"),
    "start": re.compile(r"Type START to actually run the policy \(keep the E-stop ready\):\s*$"),
    "outcome": re.compile(r"Outcome \(S=success, F=failure\)>\s*$"),
    "release": re.compile(r"type RELEASE to open the gripper:\s*$"),
    "next": re.compile(r"Q = quit safely:\s*$"),
}


def gripper_open_allowed(prompt: str | None) -> bool:
    """Allow opening while the graph is alive and autonomous motion is stopped."""
    return prompt in {"home", "open", "placed", "start", "release"}


@dataclass
class DisplayState:
    prompt: str | None = None
    trial: int | None = None
    trial_count: int | None = None
    session: int | None = None
    model: str = "-"
    condition: str = "-"
    seed: str = "-"
    position_id: str = "-"
    position: str = "-"
    repeat: str = "-"
    target_xyz: str = "-"
    grasp_xyz: str = "-"
    delta_xyz: str = "-"
    xy_error: str = "-"
    error_3d: str = "-"
    pool_progress: str = "waiting"


class EvaluationOutputParser:
    """Incrementally parse runner output without requiring newline prompts."""

    TRIAL_RE = re.compile(
        r"Trial (\d+)/(\d+) \| attempt \d+ \| session (\d+) \| "
        r"(M\d+) \(condition ([A-D]), seed (\d+)\)")
    POSITION_RE = re.compile(
        r"Evaluation position: (eval_\d+) = ([^|\r\n]+?) \| repeat (\d+/\d+)")
    TARGET_RE = re.compile(r"- (?:mean first-close .+|Q interpolation .+) XYZ: (\[[^\r\n]+\]) mm")
    GRASP_RE = re.compile(r"- Measured first-close EEF XYZ: (\[[^\r\n]+\]) mm")
    DELTA_RE = re.compile(r"- Signed error XYZ \(measured - reference\): (\[[^\r\n]+\]) mm")
    XY_RE = re.compile(r"- XY distance error: ([0-9.]+) mm")
    ERROR_3D_RE = re.compile(r"- 3-D distance error: ([0-9.]+) mm")
    OUTCOME_FINALIZED_RE = re.compile(r"Policy run reached its [0-9.]+s limit")
    POOL_LOADING_RE = re.compile(r"ACT pool loading (\d+)/(\d+): (M\d+)")
    POOL_LOADED_RE = re.compile(r"ACT pool loaded (\d+)/(\d+): (M\d+)")
    POOL_READY_RE = re.compile(r"ACT model pool ready at .+ \((\d+) models\)")
    POOL_RUNNER_READY_RE = re.compile(r"All 12 ACT models resident")

    def __init__(self) -> None:
        self.state = DisplayState()
        self._tail = ""

    def feed(self, text: str) -> DisplayState:
        window = self._tail + text.replace("\r", "")
        self._tail = window[-2_048:]

        matches = list(self.TRIAL_RE.finditer(window))
        if matches:
            match = matches[-1]
            self.state.trial = int(match.group(1))
            self.state.trial_count = int(match.group(2))
            self.state.session = int(match.group(3))
            self.state.model = match.group(4)
            self.state.condition = match.group(5)
            self.state.seed = match.group(6)
            self.state.target_xyz = "-"
            self.state.grasp_xyz = "-"
            self.state.delta_xyz = "-"
            self.state.xy_error = "-"
            self.state.error_3d = "-"

        matches = list(self.POSITION_RE.finditer(window))
        if matches:
            match = matches[-1]
            self.state.position_id = match.group(1)
            self.state.position = match.group(2).strip().split()[0]
            self.state.repeat = match.group(3)

        for pattern, attribute in (
            (self.TARGET_RE, "target_xyz"),
            (self.GRASP_RE, "grasp_xyz"),
            (self.DELTA_RE, "delta_xyz"),
            (self.XY_RE, "xy_error"),
            (self.ERROR_3D_RE, "error_3d"),
        ):
            matches = list(pattern.finditer(window))
            if matches:
                setattr(self.state, attribute, matches[-1].group(1))

        matches = list(self.POOL_LOADING_RE.finditer(window))
        if matches:
            match = matches[-1]
            self.state.pool_progress = (
                f"loading {match.group(1)}/{match.group(2)} - {match.group(3)}")
        matches = list(self.POOL_LOADED_RE.finditer(window))
        if matches:
            match = matches[-1]
            self.state.pool_progress = (
                f"loaded {match.group(1)}/{match.group(2)} - {match.group(3)}")
        matches = list(self.POOL_READY_RE.finditer(window))
        if matches:
            self.state.pool_progress = f"ready, {matches[-1].group(1)} models resident"
        elif self.POOL_RUNNER_READY_RE.search(window):
            self.state.pool_progress = "ready, 12 models resident"

        prompt_matches = []
        for name, pattern in PROMPT_PATTERNS.items():
            matches = list(pattern.finditer(window))
            if matches:
                prompt_matches.append((matches[-1].start(), name))
        latest_prompt = max(prompt_matches) if prompt_matches else None
        finalized = list(self.OUTCOME_FINALIZED_RE.finditer(window))
        if finalized and (
                latest_prompt is None or finalized[-1].start() > latest_prompt[0]):
            self.state.prompt = None
        elif latest_prompt:
            self.state.prompt = latest_prompt[1]
        return self.state

    def answer_sent(self) -> None:
        self.state.prompt = None
        self._tail = ""


class EvaluationUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Indy7 ACT evaluation")
        self.root.geometry("1240x820")
        self.root.minsize(1050, 700)
        self.root.protocol("WM_DELETE_WINDOW", self._request_close)

        self.parser = EvaluationOutputParser()
        self.output_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.process: subprocess.Popen | None = None
        self.master_fd: int | None = None
        self._closing = False
        self._policy_deadline: float | None = None

        self.progress_var = tk.StringVar(value="Idle - press Start / resume evaluation")
        self.case_var = tk.StringVar(value="Condition - | Seed - | Model -")
        self.position_var = tk.StringVar(value="Position - | Repeat -")
        self.pool_var = tk.StringVar(value="Resident model pool: waiting")
        self.status_var = tk.StringVar(value="No evaluation is running.")
        self.target_var = tk.StringVar(value="Reference XYZ: -")
        self.grasp_var = tk.StringVar(value="Measured first-close XYZ: -")
        self.delta_var = tk.StringVar(value="XYZ error (measured - reference): -")
        self.distance_var = tk.StringVar(value="XY error: -    |    3-D error: -")
        self.timer_var = tk.StringVar(value="Policy time limit: 60 s after START")

        self._build_widgets()
        self._draw_position_map(None)
        self._set_prompt(None)
        self.root.after(100, self._poll_queue)
        self.root.after(200, self._update_policy_timer)

    def _build_widgets(self) -> None:
        style = ttk.Style()
        style.configure("Title.TLabel", font=("Sans", 18, "bold"))
        style.configure("Header.TLabel", font=("Sans", 13, "bold"))
        style.configure("Big.TButton", font=("Sans", 12, "bold"), padding=10)

        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)

        header = ttk.Frame(outer)
        header.pack(fill="x")
        ttk.Label(header, text="Indy7 ACT real-robot evaluation", style="Title.TLabel").pack(
            side="left")
        ttk.Label(
            header, text="In an emergency use the physical E-STOP, not this screen.",
            foreground="#b00020", font=("Sans", 12, "bold")).pack(side="right")

        controls = ttk.Frame(outer, padding=(0, 12, 0, 8))
        controls.pack(fill="x")
        self.launch_button = ttk.Button(
            controls, text="Start / resume evaluation", style="Big.TButton",
            command=self._start_runner)
        self.launch_button.pack(side="left")
        self.abort_button = ttk.Button(
            controls, text="Abort attempt (hardware fault)", style="Big.TButton",
            command=self._interrupt_runner, state="disabled")
        self.abort_button.pack(side="left", padx=8)
        self.open_gripper_button = ttk.Button(
            controls, text="Reopen gripper", style="Big.TButton",
            command=self._open_gripper, state="disabled")
        self.open_gripper_button.pack(side="left", padx=(0, 8))
        ttk.Label(controls, textvariable=self.status_var).pack(side="left", padx=14)

        summary = ttk.LabelFrame(outer, text="Current trial", padding=10)
        summary.pack(fill="x")
        ttk.Label(summary, textvariable=self.progress_var, style="Header.TLabel").pack(anchor="w")
        ttk.Label(summary, textvariable=self.case_var, font=("Sans", 12)).pack(anchor="w", pady=2)
        ttk.Label(summary, textvariable=self.position_var, font=("Sans", 12)).pack(anchor="w")
        ttk.Label(
            summary, textvariable=self.pool_var, font=("Sans", 11, "bold"),
            foreground="#155724").pack(anchor="w", pady=(3, 0))

        main = ttk.Panedwindow(outer, orient="horizontal")
        main.pack(fill="both", expand=True, pady=(10, 0))
        left = ttk.Frame(main, padding=(0, 0, 8, 0))
        right = ttk.Frame(main, padding=(8, 0, 0, 0))
        main.add(left, weight=2)
        main.add(right, weight=3)

        map_frame = ttk.LabelFrame(left, text="Where to place the pipe", padding=8)
        map_frame.pack(fill="x")
        self.canvas = tk.Canvas(map_frame, width=390, height=310, bg="white", highlightthickness=0)
        self.canvas.pack(fill="x")

        steps = ttk.LabelFrame(left, text="Current step - only the enabled button works", padding=8)
        steps.pack(fill="x", pady=10)
        self.step_buttons: dict[str, ttk.Button] = {}
        definitions = (
            ("ready", "1. READY - workspace clear, E-stop ready", "READY"),
            ("home", "2. HOME - remove the pipe, move home", "HOME"),
            ("open", "3. OPEN - confirm the gripper really opened", "OPEN"),
            ("placed", "4. PLACED - stand the pipe on the marked spot", "PLACED"),
            ("start", "5. START - run the policy", "START"),
        )
        for row, (name, label, value) in enumerate(definitions):
            button = ttk.Button(
                steps, text=label, command=lambda item=value: self._send_answer(item))
            button.grid(row=row, column=0, sticky="ew", pady=3)
            self.step_buttons[name] = button
        steps.columnconfigure(0, weight=1)

        outcome = ttk.LabelFrame(left, text="Visual outcome", padding=8)
        outcome.pack(fill="x")
        ttk.Label(
            outcome, textvariable=self.timer_var, foreground="#b00020",
            font=("Sans", 11, "bold")).pack(fill="x", pady=(0, 6))
        button_row = ttk.Frame(outcome)
        button_row.pack(fill="x")
        self.success_button = tk.Button(
            button_row, text="Success  S", font=("Sans", 14, "bold"), bg="#2e7d32", fg="white",
            activebackground="#1b5e20", command=lambda: self._send_answer("S"))
        self.success_button.pack(side="left", fill="x", expand=True, padx=(0, 4), ipady=8)
        self.failure_button = tk.Button(
            button_row, text="Failure / stop  F", font=("Sans", 14, "bold"),
            bg="#c62828", fg="white",
            activebackground="#8e0000", command=lambda: self._send_answer("F"))
        self.failure_button.pack(side="left", fill="x", expand=True, padx=(4, 0), ipady=8)

        continuation = ttk.Frame(left)
        continuation.pack(fill="x", pady=10)
        self.next_button = ttk.Button(
            continuation, text="Continue to next trial (Enter)", style="Big.TButton",
            command=lambda: self._send_answer(""))
        self.next_button.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.quit_button = ttk.Button(
            continuation, text="Quit safely (Q)", style="Big.TButton",
            command=lambda: self._send_answer("Q"))
        self.quit_button.pack(side="left", fill="x", expand=True, padx=(4, 0))

        metrics = ttk.LabelFrame(right, text="Latest grasp position and error", padding=10)
        metrics.pack(fill="x")
        for variable in (self.target_var, self.grasp_var, self.delta_var, self.distance_var):
            ttk.Label(metrics, textvariable=variable, font=("Sans", 11)).pack(anchor="w", pady=2)

        criteria = ttk.LabelFrame(right, text="Frozen success and failure criteria", padding=8)
        criteria.pack(fill="x", pady=(10, 0))
        ttk.Label(
            criteria,
            text=(
                "Success: grasp the pipe, lift at least 50 mm, hold at least 3 s\n"
                "Immediate failure (F): likely or actual collision, knocked-over pipe, "
                "close at the wrong place, drop, lift or hold below threshold\n"
                "If a collision looks likely, press F before contact. If urgent, hit the E-STOP\n"
                "No success within 60 s means automatic failure and stop"),
            foreground="#7a0014", font=("Sans", 10, "bold"),
            justify="left", wraplength=680).pack(anchor="w")

        log_frame = ttk.LabelFrame(right, text="Runner log", padding=6)
        log_frame.pack(fill="both", expand=True, pady=(10, 0))
        self.console = scrolledtext.ScrolledText(
            log_frame, wrap="word", state="disabled", font=("Monospace", 9), bg="#111", fg="#eee")
        self.console.pack(fill="both", expand=True)

    def _draw_position_map(self, selected: str | None) -> None:
        self.canvas.delete("all")
        for first, second in (
            ("G1", "G2"), ("G2", "G3"), ("G4", "G5"), ("G5", "G6"),
            ("G7", "G8"), ("G8", "G9"), ("G1", "G4"), ("G4", "G7"),
            ("G2", "G5"), ("G5", "G8"), ("G3", "G6"), ("G6", "G9"),
        ):
            self.canvas.create_line(
                *POSITION_COORDS[first], *POSITION_COORDS[second], fill="#bbb", width=2)
        for name, (x_coord, y_coord) in POSITION_COORDS.items():
            is_selected = name == selected
            is_q = name.startswith("Q")
            radius = 24 if is_selected else 19
            fill = "#ffb300" if is_selected else ("#dceeff" if is_q else "#eeeeee")
            outline = "#b00020" if is_selected else "#555"
            self.canvas.create_oval(
                x_coord - radius, y_coord - radius, x_coord + radius, y_coord + radius,
                fill=fill, outline=outline, width=3 if is_selected else 1)
            self.canvas.create_text(
                x_coord, y_coord, text=name, font=("Sans", 12, "bold"))
        self.canvas.create_text(
            195, 295, text="Orange = where to place the pipe now", fill="#8a4b00",
            font=("Sans", 10, "bold"))

    def _start_runner(self) -> None:
        if self.process is not None and self.process.poll() is None:
            return
        if not RUNNER.is_file():
            messagebox.showerror("Cannot start", f"Runner not found:\n{RUNNER}")
            return
        self.parser = EvaluationOutputParser()
        self._render_state(self.parser.state)
        master_fd, slave_fd = pty.openpty()
        environment = os.environ.copy()
        environment["PYTHONUNBUFFERED"] = "1"
        try:
            self.process = subprocess.Popen(
                [str(RUNNER)], cwd=REPO_ROOT, stdin=slave_fd, stdout=slave_fd,
                stderr=slave_fd, env=environment, start_new_session=True,
                close_fds=True)
        except OSError as exc:
            os.close(master_fd)
            os.close(slave_fd)
            messagebox.showerror("Start failed", str(exc))
            return
        os.close(slave_fd)
        self.master_fd = master_fd
        self.launch_button.configure(state="disabled")
        self.abort_button.configure(state="normal")
        self.status_var.set("Runner started. Follow the on-screen steps.")
        self._append_console("\n===== start / resume evaluation =====\n")
        threading.Thread(target=self._read_output, daemon=True).start()

    def _read_output(self) -> None:
        decoder = codecs.getincrementaldecoder("utf-8")("replace")
        try:
            while self.master_fd is not None:
                try:
                    data = os.read(self.master_fd, 4096)
                except OSError:
                    break
                if not data:
                    break
                text = decoder.decode(data)
                if text:
                    self.output_queue.put(("output", text))
            remainder = decoder.decode(b"", final=True)
            if remainder:
                self.output_queue.put(("output", remainder))
        finally:
            process = self.process
            return_code = None if process is None else process.wait()
            self.output_queue.put(("finished", return_code))

    def _send_answer(self, answer: str) -> None:
        if self.master_fd is None or self.process is None or self.process.poll() is not None:
            return
        try:
            os.write(self.master_fd, (answer + "\n").encode("utf-8"))
        except OSError as exc:
            messagebox.showerror("Input failed", str(exc))
            return
        self.parser.answer_sent()
        self._set_prompt(None)
        self.status_var.set("Input sent. Preparing the next step.")

    def _open_gripper(self) -> None:
        if not gripper_open_allowed(self.parser.state.prompt):
            return
        if self.parser.state.prompt == "release":
            # The runner owns finalization order and opens only after logging.
            self._send_answer("RELEASE")
            return
        self.open_gripper_button.configure(state="disabled")
        self.status_var.set("Sending the gripper open command...")
        threading.Thread(target=self._run_open_gripper, daemon=True).start()

    def _run_open_gripper(self) -> None:
        try:
            result = subprocess.run(
                [str(OPEN_GRIPPER)], cwd=REPO_ROOT, text=True,
                capture_output=True, timeout=30, check=False)
            output = (result.stdout + result.stderr).strip()
            self.output_queue.put((
                "gripper_open", (result.returncode == 0, output)))
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.output_queue.put(("gripper_open", (False, str(exc))))

    def _interrupt_runner(self) -> None:
        if self.process is None or self.process.poll() is not None:
            return
        if not messagebox.askyesno(
                "Abort attempt (hardware fault)",
                "Did a hardware fault unrelated to the policy occur?\n"
                "Keep the current attempt as incomplete diagnostic data?\n\n"
                "For collision risk, a knocked-over pipe or a bad grasp, cancel and press F instead."):
            return
        self._signal_interrupt()

    def _signal_interrupt(self) -> None:
        if self.process is None or self.process.poll() is not None:
            return
        try:
            os.kill(self.process.pid, signal.SIGINT)
            self.status_var.set("Abort sent - wait for the robot and logger to shut down.")
            self.abort_button.configure(state="disabled")
        except ProcessLookupError:
            pass

    def _poll_queue(self) -> None:
        try:
            while True:
                kind, payload = self.output_queue.get_nowait()
                if kind == "output":
                    text = str(payload).replace("\r", "")
                    self._append_console(text)
                    state = self.parser.feed(text)
                    self._render_state(state)
                elif kind == "finished":
                    self._runner_finished(payload)
                elif kind == "gripper_open":
                    success, output = payload
                    self._append_console(f"\n[UI gripper open]\n{output}\n")
                    if success:
                        self.status_var.set(
                            "Open command sent - check that the fingers actually opened.")
                    else:
                        self.status_var.set("Gripper open failed - check the runner log.")
                    self._set_prompt(self.parser.state.prompt)
        except queue.Empty:
            pass
        if self.root.winfo_exists():
            self.root.after(100, self._poll_queue)

    def _render_state(self, state: DisplayState) -> None:
        if state.trial is not None and state.trial_count is not None:
            completed = state.trial - 1
            percent = 100.0 * completed / state.trial_count
            self.progress_var.set(
                f"Trial {state.trial}/{state.trial_count}  |  "
                f"{completed} done ({percent:.1f}%)  |  Session {state.session}")
        self.case_var.set(
            f"Condition {state.condition}  |  Seed {state.seed}  |  Model {state.model}")
        self.position_var.set(
            f"Position {state.position_id} = {state.position}  |  Repeat {state.repeat}")
        self.pool_var.set(f"Resident model pool: {state.pool_progress}")
        self.target_var.set(f"Reference XYZ: {state.target_xyz}")
        self.grasp_var.set(f"Measured first-close XYZ: {state.grasp_xyz}")
        self.delta_var.set(f"XYZ error (measured - reference): {state.delta_xyz}")
        self.distance_var.set(
            f"XY error: {state.xy_error} mm    |    3-D error: {state.error_3d} mm")
        self._draw_position_map(state.position if state.position in POSITION_COORDS else None)
        self._set_prompt(state.prompt)

    def _set_prompt(self, prompt: str | None) -> None:
        if prompt == "outcome" and self._policy_deadline is None:
            self._policy_deadline = time.monotonic() + POLICY_TIMEOUT_S
        elif prompt != "outcome":
            self._policy_deadline = None
        for name, button in self.step_buttons.items():
            button.configure(state="normal" if name == prompt else "disabled")
        outcome_state = "normal" if prompt == "outcome" else "disabled"
        self.success_button.configure(state=outcome_state)
        self.failure_button.configure(state=outcome_state)
        continuation_state = "normal" if prompt == "next" else "disabled"
        self.next_button.configure(state=continuation_state)
        self.quit_button.configure(state=continuation_state)
        open_state = "normal" if gripper_open_allowed(prompt) else "disabled"
        open_label = (
            "Open gripper and release the pipe" if prompt == "release" else "Reopen gripper")
        self.open_gripper_button.configure(state=open_state, text=open_label)
        labels = {
            "ready": "Check the workspace and the physical E-stop, then press READY.",
            "home": "Remove the pipe, then press HOME - the robot will move.",
            "open": "Press OPEN only once the gripper is visibly fully open.",
            "placed": "Stand the pipe upright on the orange spot, then press PLACED.",
            "start": "With the E-stop ready, press START to run the policy.",
            "outcome": (
                "If a collision looks likely, stop it with F before contact. "
                "If it is urgent, use the physical E-stop."),
            "release": (
                "Recording finished. Support the pipe by hand, then press "
                "'Open gripper and release the pipe'."),
            "next": "Saved and validated. Continue to the next trial, or quit.",
        }
        if prompt in labels:
            self.status_var.set(labels[prompt])

    def _update_policy_timer(self) -> None:
        if self._policy_deadline is None:
            self.timer_var.set("Policy time limit: 60 s after START")
        else:
            remaining = max(0.0, self._policy_deadline - time.monotonic())
            self.timer_var.set(f"Policy time remaining: {remaining:04.1f} s")
        if self.root.winfo_exists():
            self.root.after(200, self._update_policy_timer)

    def _append_console(self, text: str) -> None:
        self.console.configure(state="normal")
        self.console.insert("end", text)
        try:
            line_count = int(self.console.index("end-1c").split(".")[0])
            if line_count > MAX_CONSOLE_LINES:
                self.console.delete("1.0", f"{line_count - MAX_CONSOLE_LINES}.0")
        except (ValueError, tk.TclError):
            pass
        self.console.see("end")
        self.console.configure(state="disabled")

    def _runner_finished(self, return_code: object) -> None:
        if self.master_fd is not None:
            try:
                os.close(self.master_fd)
            except OSError:
                pass
        self.master_fd = None
        self.abort_button.configure(state="disabled")
        self.launch_button.configure(state="normal")
        self._set_prompt(None)
        if return_code == 0:
            self.status_var.set("Runner exited cleanly. You can resume later.")
        else:
            self.status_var.set(
                f"Runner exited (code={return_code}). Check the log.")
        self.process = None
        if self._closing:
            self.root.destroy()

    def _request_close(self) -> None:
        if self.process is not None and self.process.poll() is None:
            if not messagebox.askyesno(
                    "Quit", "An evaluation is running. Abort the current attempt and\n"
                    "shut down the robot and logger before quitting?"):
                return
            self._closing = True
            self._signal_interrupt()
            return
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    EvaluationUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
