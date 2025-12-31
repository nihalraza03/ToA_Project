from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Tuple, Set, Optional, List

Move = str
State = str
Symbol = str
TransitionKey = Tuple[State, Symbol]
TransitionVal = Tuple[State, Symbol, Move]


# ---------------- Tape ----------------
class Tape:
    def __init__(self, input_str: str, blank: str = "_"):
        self.blank = blank
        self.cells: Dict[int, str] = {i: ch for i, ch in enumerate(input_str)}

    def read(self, i: int) -> str:
        return self.cells.get(i, self.blank)

    def write(self, i: int, sym: str) -> None:
        if sym == self.blank:
            self.cells.pop(i, None)
        else:
            self.cells[i] = sym

    def bounds(self, include: Optional[int] = None) -> Tuple[int, int]:
        keys = list(self.cells.keys())
        if include is not None:
            keys.append(include)
        if not keys:
            return (0, 0)
        return (min(keys), max(keys))

    def render(self, head: int, pad: int = 12) -> str:
        lo, hi = self.bounds(include=head)
        lo -= pad
        hi += pad
        out = []
        for i in range(lo, hi + 1):
            ch = self.read(i)
            out.append(f"[{ch}]" if i == head else f" {ch} ")
        return "".join(out)

    def content_str(self) -> str:
        lo, hi = self.bounds()
        return "".join(self.read(i) for i in range(lo, hi + 1)).strip(self.blank)


# --------------- TM Engine ---------------
@dataclass
class TMResult:
    halted: bool
    accepted: bool
    steps: int
    final_state: str
    tape: Tape
    head: int
    reason: str


class TuringMachine:
    def __init__(
        self,
        *,
        transitions: Dict[TransitionKey, TransitionVal],
        start_state: str,
        accept_states: Set[str],
        reject_states: Set[str],
        blank: str = "_",
        max_steps: int = 500_000,
        trace: bool = False,
    ):
        self.transitions = transitions
        self.start_state = start_state
        self.accept_states = set(accept_states)
        self.reject_states = set(reject_states)
        self.blank = blank
        self.max_steps = max_steps
        self.trace = trace

        self.state = start_state
        self.head = 0
        self.tape = Tape("", blank=blank)
        self.steps = 0

    def load_input(self, s: str, head_at: int = 0) -> None:
        self.tape = Tape(s, blank=self.blank)
        self.head = head_at
        self.state = self.start_state
        self.steps = 0

    def step(self) -> TMResult:
        if self.state in self.accept_states:
            return TMResult(True, True, self.steps, self.state, self.tape, self.head, "accept")
        if self.state in self.reject_states:
            return TMResult(True, False, self.steps, self.state, self.tape, self.head, "reject")
        if self.steps >= self.max_steps:
            return TMResult(True, False, self.steps, self.state, self.tape, self.head, "max steps exceeded")

        sym = self.tape.read(self.head)
        key = (self.state, sym)
        if key not in self.transitions:
            self.state = next(iter(self.reject_states)) if self.reject_states else "__REJECT__"
            return TMResult(True, False, self.steps, self.state, self.tape, self.head, "missing transition")

        new_state, write_sym, move = self.transitions[key]

        if self.trace:
            print(
                f"step={self.steps:05d} state={self.state:>14} head={self.head:>4} "
                f"read={sym!r} -> write={write_sym!r}, move={move}, next={new_state}"
            )
            print(self.tape.render(self.head))

        self.tape.write(self.head, write_sym)
        if move == "L":
            self.head -= 1
        elif move == "R":
            self.head += 1
        elif move == "S":
            pass
        else:
            raise ValueError(f"Invalid move: {move}")

        self.state = new_state
        self.steps += 1
        return TMResult(False, False, self.steps, self.state, self.tape, self.head, "running")

    def run(self) -> TMResult:
        while True:
            r = self.step()
            if r.halted:
                return r


# --------------- TM Program: UART 8E1 Validator ---------------
def build_uart_8e1_validator_tm():
    """
    Validates a stream of frames separated by '|'.

    Frame format (11 bits): 0 d0 d1 d2 d3 d4 d5 d6 d7 p 1
    - start bit must be 0
    - stop bit must be 1
    - even parity: p chosen so total 1s in (data + p) is even

    Tape behavior (important for viva):
    - head moves RIGHT reading the frame
    - then moves LEFT back to the start and overwrites the start cell with:
        K = frame OK
        E = frame ERROR
    """
    qa, qr = "qa", "qr"
    T: Dict[TransitionKey, TransitionVal] = {}

    def add(st, sym, nst, wsym, mv):
        T[(st, sym)] = (nst, wsym, mv)

    # SEEK: skip noise and previously marked frames; find next start bit '0'
    for sym in ["1", "|", "K", "E"]:
        add("q_seek", sym, "q_seek", sym, "R")
    add("q_seek", "0", "q_d0_even", "s", "R")  # mark start temporarily as 's'
    add("q_seek", "_", qa, "_", "S")

    # Read 8 data bits; track parity of ones in state
    def next_state(i: int, parity: str) -> str:
        if i == 7:
            return f"q_parity_{parity}"
        return f"q_d{i+1}_{parity}"

    for i in range(8):
        for parity in ["even", "odd"]:
            st = f"q_d{i}_{parity}"
            add(st, "0", next_state(i, parity), "0", "R")
            toggled = "odd" if parity == "even" else "even"
            add(st, "1", next_state(i, toggled), "1", "R")

            # malformed stream
            for bad in ["|", "_", "K", "E", "s"]:
                add(st, bad, "q_back_err", "E", "L")

    # Parity check for even parity
    add("q_parity_even", "0", "q_stop_ok", "0", "R")
    add("q_parity_even", "1", "q_stop_err", "1", "R")
    add("q_parity_odd",  "1", "q_stop_ok", "1", "R")
    add("q_parity_odd",  "0", "q_stop_err", "0", "R")

    # Stop bit check
    add("q_stop_ok", "1", "q_back_ok", "1", "L")
    for bad in ["0", "|", "_", "K", "E", "s"]:
        add("q_stop_ok", bad, "q_back_err", bad if bad != "_" else "E", "L")

    # If parity already wrong, always error regardless of stop
    for sym in ["0", "1", "|", "_", "K", "E", "s"]:
        add("q_stop_err", sym, "q_back_err", sym if sym != "_" else "E", "L")

    # Move LEFT back to 's' then overwrite 's' with K/E
    for sym in ["0", "1", "|", "K", "E"]:
        add("q_back_ok", sym, "q_back_ok", sym, "L")
        add("q_back_err", sym, "q_back_err", sym, "L")
    add("q_back_ok", "s", "q_to_delim", "K", "R")
    add("q_back_err", "s", "q_to_delim", "E", "R")
    add("q_back_ok", "_", qa, "_", "S")
    add("q_back_err", "_", qa, "_", "S")

    # Move RIGHT to '|' then continue
    for sym in ["0", "1", "K", "E", "s"]:
        add("q_to_delim", sym, "q_to_delim", sym, "R")
    add("q_to_delim", "|", "q_seek", "|", "R")
    add("q_to_delim", "_", qa, "_", "S")

    return dict(
        transitions=T,
        start_state="q_seek",
        accept_states={qa},
        reject_states={qr},
        blank="_",
    )


# --------------- Log I/O ---------------
def load_log_lines(path: str) -> List[str]:
    out = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            out.append(line)
    return out


def extract_raw(line: str) -> str:
    return line.split("raw=", 1)[1].strip() if "raw=" in line else ""


def write_validation_report(out_path: str, selected_line: str, raw: str, marked: str, steps: int) -> None:
    ok = marked.count("K")
    err = marked.count("E")
    total = ok + err
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("IoT UART Bitstream Validation Report (TM-based)\n\n")
        f.write("INPUT LOG LINE:\n")
        f.write(selected_line + "\n\n")
        f.write("RAW BITSTREAM:\n")
        f.write(raw + "\n\n")
        f.write("TM-MARKED BITSTREAM:\n")
        f.write(marked + "\n\n")
        f.write(f"SUMMARY: total_frames={total}, ok={ok}, errors={err}, tm_steps={steps}\n")
        f.write("LEGEND: K=start cell overwritten after OK frame; E=start cell overwritten after ERROR frame; |=separator\n")


# --------------- Console UI ---------------
def main():
    print("IoT UART Bitstream Log Validator (Turing Machine)")
    log_path = "iot_capture.log"
    # log_path = input("Enter log file path (default iot_capture.log): ").strip() or "iot_capture.log"
    lines = load_log_lines(log_path)
    if not lines:
        print("No log entries found.")
        return

    print("\nAvailable log entries:")
    for i, ln in enumerate(lines, 1):
        print(f"{i}) {ln}")

    idx = int(input("\nPick entry number: ").strip())
    chosen = lines[idx - 1]
    raw = extract_raw(chosen)
    if not raw:
        print("Selected entry has no raw= field.")
        return

    trace = input("Show TM step-by-step trace (read/write/move)? (y/n): ").strip().lower() == "y"

    spec = build_uart_8e1_validator_tm()
    tm = TuringMachine(
        transitions=spec["transitions"],
        start_state=spec["start_state"],
        accept_states=spec["accept_states"],
        reject_states=spec["reject_states"],
        blank=spec["blank"],
        max_steps=500_000,
        trace=trace,
    )

    tm.load_input(raw)
    res = tm.run()

    marked = res.tape.content_str()

    print("\nRAW stream:")
    print(raw)
    print("\nTM-marked stream (frame start cell becomes K/E after validation):")
    print(marked)

    out_file = "validated_output.log"
    write_validation_report(out_file, chosen, raw, marked, res.steps)
    print(f"\nSaved validation report: {out_file}")


if __name__ == "__main__":
    main()
