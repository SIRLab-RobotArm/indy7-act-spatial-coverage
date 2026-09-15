#!/usr/bin/env python3
"""Desktop UI for choosing and running one ACT model without recording."""

from __future__ import annotations

import codecs
from contextlib import suppress
import os
from pathlib import Path
import pty
import queue
import re
import signal
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from scripts.eval.run_selected_model import POSITION_NAMES, model_registry


RUNNER = REPO_ROOT / "scripts/eval/run_selected_model.sh"
OPEN_GRIPPER = REPO_ROOT / "scripts/robot/open_gripper.sh"
PROMPT_RE = re.compile(r"DIRECT_RUN_PROMPT:(ready|home|open|placed|start|stop|release)")
POSITION_COORDS = {
    "G1": (55, 45), "G2": (195, 45), "G3": (335, 45),
    "G4": (55, 155), "G5": (195, 155), "G6": (335, 155),
    "G7": (55, 265), "G8": (195, 265), "G9": (335, 265),
    "Q1": (125, 100), "Q2": (265, 100),
    "Q3": (125, 210), "Q4": (265, 210),
}


class DirectRunOutputParser:
    def __init__(self) -> None:
        self.prompt: str | None = None
        self._tail = ""

    def feed(self, text: str) -> str | None:
        window = self._tail + text.replace("\r", "")
        self._tail = window[-256:]
        matches = list(PROMPT_RE.finditer(window))
        if matches:
            self.prompt = matches[-1].group(1)
        if "DIRECT_RUN_DONE" in window:
            self.prompt = None
        return self.prompt

    def answer_sent(self) -> None:
        self.prompt = None
        self._tail = ""


class ModelRunUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Indy7 ACT direct model run")
        self.root.geometry("1080x760")
        self.root.minsize(960, 680)
        self.root.protocol("WM_DELETE_WINDOW", self._request_close)

        self.registry = model_registry()
        self.model_labels = {
            f"{code} — Condition {entry['condition']} / Seed {entry['seed']}": code
            for code, entry in self.registry.items()
        }
        self.parser = DirectRunOutputParser()
        self.output_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.process: subprocess.Popen | None = None
        self.master_fd: int | None = None
        self._closing = False

        first_label = next(iter(self.model_labels))
        self.model_var = tk.StringVar(value=first_label)
        self.position_var = tk.StringVar(value="G5")
        self.selection_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Pick a model and a position, then press Prepare run.")

        self._build_widgets()
        self._draw_position_map()
        self._update_selection()
        self._set_prompt(None)
        self.root.after(100, self._poll_queue)

    def _build_widgets(self) -> None:
        style = ttk.Style()
        style.configure("Title.TLabel", font=("Sans", 18, "bold"))
        style.configure("Big.TButton", font=("Sans", 12, "bold"), padding=10)

        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)
        header = ttk.Frame(outer)
        header.pack(fill="x")
        ttk.Label(header, text="Indy7 ACT direct model run", style="Title.TLabel").pack(side="left")
        ttk.Label(
            header, text="Nothing is recorded | in an emergency use the physical E-STOP",
            foreground="#b00020", font=("Sans", 11, "bold")).pack(side="right")

        chooser = ttk.LabelFrame(outer, text="1. Choose a model and a pipe position", padding=10)
        chooser.pack(fill="x", pady=(12, 8))
        ttk.Label(chooser, text="Model").grid(row=0, column=0, sticky="w")
        self.model_box = ttk.Combobox(
            chooser, textvariable=self.model_var, values=list(self.model_labels),
            state="readonly", width=38)
        self.model_box.grid(row=0, column=1, sticky="w", padx=(8, 18))
        self.model_box.bind("<<ComboboxSelected>>", lambda _event: self._update_selection())
        ttk.Label(chooser, text="Position").grid(row=0, column=2, sticky="w")
        self.position_box = ttk.Combobox(
            chooser, textvariable=self.position_var, values=POSITION_NAMES,
            state="readonly", width=8)
        self.position_box.grid(row=0, column=3, sticky="w", padx=8)
        self.position_box.bind("<<ComboboxSelected>>", lambda _event: self._position_changed())
        ttk.Label(
            chooser, textvariable=self.selection_var,
            font=("Sans", 11, "bold")).grid(row=1, column=0, columnspan=4, sticky="w", pady=(8, 0))

        controls = ttk.Frame(outer)
        controls.pack(fill="x", pady=(0, 8))
        self.launch_button = ttk.Button(
            controls, text="Prepare run", style="Big.TButton", command=self._start_runner)
        self.launch_button.pack(side="left")
        self.abort_button = ttk.Button(
            controls, text="Stop everything safely", style="Big.TButton",
            command=self._interrupt_runner, state="disabled")
        self.abort_button.pack(side="left", padx=8)
        ttk.Label(controls, textvariable=self.status_var).pack(side="left", padx=8)

        main = ttk.Panedwindow(outer, orient="horizontal")
        main.pack(fill="both", expand=True)
        left = ttk.Frame(main, padding=(0, 0, 8, 0))
        right = ttk.Frame(main, padding=(8, 0, 0, 0))
        main.add(left, weight=2)
        main.add(right, weight=3)

        map_frame = ttk.LabelFrame(left, text="Pipe position - click a circle to choose", padding=8)
        map_frame.pack(fill="x")
        self.canvas = tk.Canvas(map_frame, width=390, height=310, bg="white", highlightthickness=0)
        self.canvas.pack(fill="x")

        steps = ttk.LabelFrame(left, text="2. Safety sequence", padding=8)
        steps.pack(fill="x", pady=10)
        definitions = (
            ("ready", "1. READY - workspace and E-stop checked", "READY"),
            ("home", "2. HOME - remove the pipe, move home", "HOME"),
            ("open", "3. OPEN - confirm the gripper opened", "OPEN"),
            ("placed", "4. PLACED - place the pipe at the chosen position", "PLACED"),
            ("start", "5. START - run the chosen policy", "START"),
        )
        self.step_buttons: dict[str, ttk.Button] = {}
        for row, (name, label, answer) in enumerate(definitions):
            button = ttk.Button(steps, text=label, command=lambda value=answer: self._send_answer(value))
            button.grid(row=row, column=0, sticky="ew", pady=3)
            self.step_buttons[name] = button
        steps.columnconfigure(0, weight=1)
        self.stop_button = tk.Button(
            steps, text="Stop policy  STOP", font=("Sans", 13, "bold"),
            bg="#c62828", fg="white", command=lambda: self._send_answer("STOP"))
        self.stop_button.grid(row=5, column=0, sticky="ew", pady=(8, 3), ipady=6)
        self.release_button = ttk.Button(
            steps, text="6. RELEASE - support the pipe and open the gripper",
            command=lambda: self._send_answer("RELEASE"))
        self.release_button.grid(row=6, column=0, sticky="ew", pady=3)

        utility = ttk.Frame(left)
        utility.pack(fill="x")
        self.open_button = ttk.Button(
            utility, text="Reopen gripper", command=self._open_gripper, state="disabled")
        self.open_button.pack(fill="x")

        log_frame = ttk.LabelFrame(right, text="Runner log (not saved as a result)", padding=6)
        log_frame.pack(fill="both", expand=True)
        self.console = scrolledtext.ScrolledText(
            log_frame, wrap="word", state="disabled", font=("Monospace", 9),
            bg="#111", fg="#eee")
        self.console.pack(fill="both", expand=True)

    def _position_changed(self) -> None:
        self._draw_position_map()
        self._update_selection()

    def _select_position(self, position: str) -> None:
        if self.process is not None and self.process.poll() is None:
            return
        self.position_var.set(position)
        self._position_changed()

    def _draw_position_map(self) -> None:
        self.canvas.delete("all")
        for first, second in (
            ("G1", "G2"), ("G2", "G3"), ("G4", "G5"), ("G5", "G6"),
            ("G7", "G8"), ("G8", "G9"), ("G1", "G4"), ("G4", "G7"),
            ("G2", "G5"), ("G5", "G8"), ("G3", "G6"), ("G6", "G9"),
        ):
            self.canvas.create_line(*POSITION_COORDS[first], *POSITION_COORDS[second], fill="#bbb", width=2)
        selected = self.position_var.get()
        for name, (x_coord, y_coord) in POSITION_COORDS.items():
            active = name == selected
            radius = 24 if active else 19
            tag = f"position_{name}"
            self.canvas.create_oval(
                x_coord - radius, y_coord - radius, x_coord + radius, y_coord + radius,
                fill="#ffb300" if active else ("#dceeff" if name.startswith("Q") else "#eee"),
                outline="#b00020" if active else "#555", width=3 if active else 1,
                tags=(tag,))
            self.canvas.create_text(
                x_coord, y_coord, text=name, font=("Sans", 12, "bold"), tags=(tag,))
            self.canvas.tag_bind(tag, "<Button-1>", lambda _event, value=name: self._select_position(value))
        self.canvas.create_text(
            195, 295, text="Orange = the chosen pipe position", fill="#8a4b00",
            font=("Sans", 10, "bold"))

    def _update_selection(self) -> None:
        code = self.model_labels.get(self.model_var.get(), "-")
        self.selection_var.set(
            f"Selected: model {code}  |  position {self.position_var.get()}  |  nothing saved")

    def _start_runner(self) -> None:
        if self.process is not None and self.process.poll() is None:
            return
        model_code = self.model_labels.get(self.model_var.get())
        position = self.position_var.get()
        if model_code is None or position not in POSITION_NAMES:
            messagebox.showerror("Invalid selection", "Choose a model and a position again.")
            return
        self.parser = DirectRunOutputParser()
        master_fd, slave_fd = pty.openpty()
        environment = os.environ.copy()
        environment["PYTHONUNBUFFERED"] = "1"
        try:
            self.process = subprocess.Popen(
                [str(RUNNER), "--model", model_code, "--position", position],
                cwd=REPO_ROOT, stdin=slave_fd, stdout=slave_fd, stderr=slave_fd,
                env=environment, start_new_session=True, close_fds=True)
        except OSError as exc:
            os.close(master_fd)
            os.close(slave_fd)
            messagebox.showerror("Start failed", str(exc))
            return
        os.close(slave_fd)
        self.master_fd = master_fd
        self.launch_button.configure(state="disabled")
        self.abort_button.configure(state="normal")
        self.model_box.configure(state="disabled")
        self.position_box.configure(state="disabled")
        self.status_var.set(f"Preparing the run for {model_code} / {position}.")
        self._append_console(f"\n===== direct run {model_code} / {position} (nothing saved) =====\n")
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
        self.status_var.set("Input sent. Wait for the next instruction.")

    def _open_gripper(self) -> None:
        if self.parser.prompt not in {"home", "open", "placed", "start"}:
            return
        self.open_button.configure(state="disabled")
        threading.Thread(target=self._run_open_gripper, daemon=True).start()

    def _run_open_gripper(self) -> None:
        try:
            result = subprocess.run(
                [str(OPEN_GRIPPER)], cwd=REPO_ROOT, text=True,
                capture_output=True, timeout=30, check=False)
            self.output_queue.put(("gripper", (result.returncode == 0, result.stdout + result.stderr)))
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.output_queue.put(("gripper", (False, str(exc))))

    def _set_prompt(self, prompt: str | None) -> None:
        for name, button in self.step_buttons.items():
            button.configure(state="normal" if name == prompt else "disabled")
        self.stop_button.configure(state="normal" if prompt == "stop" else "disabled")
        self.release_button.configure(state="normal" if prompt == "release" else "disabled")
        self.open_button.configure(
            state="normal" if prompt in {"home", "open", "placed", "start"} else "disabled")
        messages = {
            "ready": "Check the workspace and the physical E-stop, then press READY.",
            "home": "Remove the pipe, then press HOME. The robot will move.",
            "open": "Check that the gripper really opened, then press OPEN.",
            "placed": f"Place the pipe at {self.position_var.get()}, then press PLACED.",
            "start": "With the E-stop ready, press START to run the chosen model.",
            "stop": "The policy is running. Press STOP at any time (automatic stop after 60 s).",
            "release": "Support the pipe by hand, then press RELEASE.",
        }
        if prompt in messages:
            self.status_var.set(messages[prompt])

    def _poll_queue(self) -> None:
        try:
            while True:
                kind, payload = self.output_queue.get_nowait()
                if kind == "output":
                    text = str(payload).replace("\r", "")
                    self._append_console(text)
                    self._set_prompt(self.parser.feed(text))
                elif kind == "finished":
                    self._runner_finished(payload)
                elif kind == "gripper":
                    success, output = payload
                    self._append_console(f"\n[gripper open]\n{output}\n")
                    self.status_var.set("Gripper opened" if success else "Gripper open failed - check the log")
                    self._set_prompt(self.parser.prompt)
        except queue.Empty:
            pass
        if self.root.winfo_exists():
            self.root.after(100, self._poll_queue)

    def _append_console(self, text: str) -> None:
        self.console.configure(state="normal")
        self.console.insert("end", text)
        self.console.see("end")
        self.console.configure(state="disabled")

    def _interrupt_runner(self) -> None:
        if self.process is None or self.process.poll() is not None:
            return
        if messagebox.askyesno(
                "Stop everything", "Stop the policy and shut down the ROS and ACT processes this UI started?"):
            with suppress(ProcessLookupError):
                os.kill(self.process.pid, signal.SIGINT)
            self.abort_button.configure(state="disabled")
            self.status_var.set("Stop requested - wait for the robot and processes to shut down.")

    def _runner_finished(self, return_code: object) -> None:
        if self.master_fd is not None:
            try:
                os.close(self.master_fd)
            except OSError:
                pass
        self.master_fd = None
        self.process = None
        self.launch_button.configure(state="normal")
        self.abort_button.configure(state="disabled")
        self.model_box.configure(state="readonly")
        self.position_box.configure(state="readonly")
        self._set_prompt(None)
        self.status_var.set(
            "Run complete - nothing was saved." if return_code == 0
            else f"Runner exited (code={return_code}) - check the log.")
        if self._closing:
            self.root.destroy()

    def _request_close(self) -> None:
        if self.process is not None and self.process.poll() is None:
            if not messagebox.askyesno("Quit", "Stop the run safely and close this window?"):
                return
            self._closing = True
            with suppress(ProcessLookupError):
                os.kill(self.process.pid, signal.SIGINT)
            return
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    ModelRunUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
