"""Narration script + storyboard beats: the single source of truth for timing.

Every scene's duration is the sum of its beat durations.  script.md,
narration.srt and storyboard.md are generated from this file
(`python3 tools/make_docs.py`), and every scene program reads its beat
start/end times from here, so the picture always lines up with the words.

Speaking rate is 140 words/min.  Narration for a beat starts LEAD seconds after
the beat starts; `check()` asserts that it finishes before the beat ends.
Durations are multiples of 0.5 s (12 frames at 24 fps).
"""
from __future__ import annotations

from dataclasses import dataclass, field

WPM = 140.0
LEAD = 0.25          # s of silence at the start of each beat before narration
FPS = 24


@dataclass
class Beat:
    id: str
    dur: float
    text: str
    visual: str
    start: float = 0.0       # filled in: seconds from scene start
    gstart: float = 0.0      # filled in: seconds from video start

    @property
    def end(self):
        return self.start + self.dur

    @property
    def gend(self):
        return self.gstart + self.dur

    @property
    def words(self):
        return len(self.text.replace("—", " ").split())

    @property
    def speak_time(self):
        return self.words / WPM * 60.0

    def word_time(self, word, occurrence=1, scene_relative=True):
        """Time (s) at which `word` (case-insensitive, punctuation ignored;
        may be a multi-word phrase) starts being spoken, at WPM from LEAD.
        Scene-relative by default (add Scene.start for global time)."""
        import re
        toks = [re.sub(r"[^\w'-]", "", w).lower() for w in self.text.replace("—", " ").split()]
        target = [re.sub(r"[^\w'-]", "", w).lower() for w in word.split()]
        seen = 0
        for i in range(len(toks) - len(target) + 1):
            if toks[i:i + len(target)] == target:
                seen += 1
                if seen == occurrence:
                    t = LEAD + i * 60.0 / WPM
                    return (self.start if scene_relative else self.gstart) + t
        raise KeyError(f"word {word!r} not in beat {self.id!r}")


@dataclass
class Scene:
    id: str
    title: str
    beats: list
    summary: str = ""
    start: float = 0.0       # global start (s)

    @property
    def dur(self):
        return sum(b.dur for b in self.beats)

    @property
    def frames(self):
        return int(round(self.dur * FPS))

    def beat(self, bid):
        for b in self.beats:
            if b.id == bid:
                return b
        raise KeyError(f"{self.id}: no beat {bid!r}")

    def __getitem__(self, bid):
        return self.beat(bid)


SCENES = [
    Scene("s01", "Overview", summary="The whole car, then the power path from engine to wheels.", beats=[
        Beat("car", 7.0,
             "This is a front-engine, rear-wheel-drive car with a five-speed manual gearbox.",
             "Studio. Opaque car, 3/4 front-left view, slow orbit at eye level (50 mm). Title "
             "'How a Manual Car Works' fades in and out. Car parked, engine off: drivetrain static."),
        Beat("inside", 7.0,
             "Let's look inside, at the parts that turn burning fuel into turning wheels.",
             "Camera pushes in over the front wing. Body fades from opaque to a faint x-ray shell; the "
             "camera passes through the bodywork and the whole drivetrain is revealed."),
        Beat("path", 15.0,
             "Power starts in the engine. It flows through the clutch and gearbox, along the propeller "
             "shaft to the differential at the rear axle, and out through two driveshafts to the wheels.",
             "High 3/4 view travelling front to rear along the drivetrain. A warm glow sweeps the power "
             "path, each part lighting and getting its label as it is named: Engine, Clutch, Gearbox, "
             "Propeller shaft, Differential, Driveshafts, Rear wheels."),
        Beat("follow", 3.0,
             "Let's follow it.",
             "Camera dives toward the engine; body and chassis fade away; hand-off to the engine scene."),
    ]),
    Scene("s02", "The engine", summary="Inline-4, four-stroke cycle, valvetrain, firing order, flywheel.", beats=[
        Beat("inline4", 8.0,
             "The engine is an inline four: four cylinders in a row, each with a sliding piston.",
             "Engine alone, dark studio. Block and head cut away down the cylinder centreline. Slow orbit. "
             "Labels: cylinder numbers 1-4 (1 at the front), Piston. Engine idling 850 rpm shown ~200x slowed."),
        Beat("crank", 7.0,
             "Connecting rods link the pistons to the crankshaft, turning up-and-down motion into rotation.",
             "Lower angle on the bottom end through the cutaway. Rods and crankshaft highlighted. Labels: "
             "Connecting rod, Crankshaft."),
        Beat("cycle", 6.0,
             "Each cylinder repeats a four-stroke cycle over two crankshaft turns.",
             "Push in to a cross-section of cylinder 1 (valves, ports, spark plug, piston). A 4-box stroke "
             "strip appears. Cylinder 1 reaches TDC at the end of this beat."),
        Beat("intake", 7.0,
             "Intake: the piston moves down, drawing air and fuel in through the open intake valves.",
             "Cylinder 1, crank 0-180 deg of its intake stroke. Intake valves open, cool blue charge fills the "
             "cylinder. Stroke strip: INTAKE. Label: Intake valve."),
        Beat("compression", 7.0,
             "Compression: the intake valves close, and the rising piston squeezes the mixture.",
             "Intake valves close ~50 deg after BDC (~1.9 s in); then the piston rises with both valves shut and the charge becomes denser. Spark plug fires ~15 deg before TDC at the "
             "very end. Strip: COMPRESSION. Label: Spark plug."),
        Beat("power", 7.0,
             "Power: a spark ignites the mixture, and the hot gas forces the piston down.",
             "Combustion glow (orange) fills the chamber and fades as the piston is driven down. Strip: POWER."),
        Beat("exhaust", 7.0,
             "Exhaust: with the exhaust valves open, the rising piston pushes the burnt gas out.",
             "Exhaust valves (already opening near the end of the power stroke) are open; grey-brown gas "
             "leaves through the exhaust port. Strip: EXHAUST. Label: Exhaust valve."),
        Beat("valvetrain", 12.5,
             "Camshafts open the valves. A timing chain drives them from the crankshaft at exactly half its "
             "speed, because each valve opens only once every two turns.",
             "Camera rises and orbits to the front. Cam cover removed, timing cover cut away. Labels: Intake "
             "camshaft, Exhaust camshaft, Timing chain, Crank sprocket 21T, Cam sprocket 42T. HUD: crank and "
             "cam rpm (850 / 425)."),
        Beat("firing", 12.5,
             "Only the power stroke drives the crankshaft, so the cylinders take turns, firing in the order "
             "one, three, four, two: a power stroke every half turn.",
             "Side view of all four cylinders. Each combustion flash happens in turn; overlay ticker "
             "1 > 3 > 4 > 2 highlights the cylinder now firing."),
        Beat("flywheel", 8.5,
             "At the back, a heavy flywheel smooths out the pulses. Its face is one half of the clutch.",
             "Camera travels to the rear of the engine: flywheel with starter ring gear, ending on its "
             "friction face. Label: Flywheel."),
    ]),
    Scene("s03", "The clutch", summary="Parts, splines, engaged, released via hydraulics, slipping take-off.", beats=[
        Beat("parts", 11.0,
             "Between the engine and the gearbox sits the clutch: the flywheel, a friction disc, and a "
             "pressure plate with a diaphragm spring.",
             "Exploded view along the crank axis: flywheel, friction disc, pressure plate + diaphragm spring "
             "+ cover, release bearing, input shaft. Parts slide together and assemble. Labels on each part."),
        Beat("splines", 7.5,
             "The disc's hub is splined to the gearbox input shaft, so they always turn together.",
             "Close-up of the disc hub on the input-shaft splines. Labels: Splines, Input shaft."),
        Beat("engaged", 11.0,
             "With the pedal up, the spring clamps the disc between the pressure plate and flywheel, so the "
             "input shaft turns with the engine.",
             "Half-section of the assembled clutch, everything turning together (gearbox in neutral, engine "
             "idling). HUD: CLUTCH ENGAGED, engine rpm = input-shaft rpm."),
        Beat("release", 19.0,
             "Press the pedal, and fluid flows from the master cylinder to the slave cylinder. It pushes the "
             "release fork and bearing against the spring's fingers. The spring flexes, the pressure plate "
             "pulls back, and the disc is free.",
             "Pull back to show pedal, master cylinder, hydraulic line, slave cylinder, release fork. Pedal "
             "goes down; a pulse runs along the line; fork and bearing move; fingers deflect; pressure plate "
             "lifts. Disc slows (no longer driven). At the end, first gear is selected (HUD) and the disc stops."),
        Beat("slip", 15.0,
             "To pull away, the pedal comes up slowly. The disc slips between the flywheel and pressure plate, "
             "speeding up as it passes on torque, until it matches the engine and locks.",
             "Pedal rises; HUD shows engine rpm and disc rpm converging; status SLIPPING then ENGAGED; car "
             "starts to roll in 1st."),
    ]),
    Scene("s04", "The gearbox", summary="Three shafts, constant mesh, synchronisers, linkage, ratios, reverse.", beats=[
        Beat("shafts", 13.0,
             "The gearbox has three shafts. The input shaft, driven by the clutch, turns the countershaft below. "
             "The output shaft runs out the back, in line with the input.",
             "Gearbox case becomes a cutaway. Labels: Input shaft, Countershaft, Output shaft. Neutral, clutch "
             "engaged, engine idling, car stationary (output shaft still)."),
        Beat("neutral", 16.5,
             "The countershaft's forward gears each mesh with a gear on the output shaft. They're always in mesh, "
             "but the output gears spin freely on bearings. Until one is locked to the shaft, no power gets "
             "through: neutral.",
             "Slow track along the gear train. All gears turn, but the output shaft and synchro hubs stand "
             "still. Label: Free-spinning gears. HUD gear N."),
        Beat("synchro", 16.0,
             "Synchronizers do the locking. A hub is splined to the shaft, and a sleeve slides on it. Each gear "
             "carries dog teeth and a cone, with a brass blocker ring between cone and sleeve.",
             "Close-up on the 1-2 synchroniser; exploded view: hub, sleeve, blocker ring, cone, dog teeth; "
             "reassembles. Clutch pressed at the start of this beat (HUD)."),
        Beat("lock", 7.5,
             "Slide the sleeve over the dog teeth, and the gear is locked to the shaft.",
             "Sleeve moves toward 1st: blocker ring stops the gear train (output is stationary), sleeve slides "
             "over the dog teeth. HUD gear 1."),
        Beat("linkage", 10.5,
             "Forks on shift rails move the sleeves. Moving the lever sideways picks a rail; forward or back "
             "slides it.",
             "Pull back: gear lever, selector finger, three rails and forks. Lever moves across the gate and "
             "back to 1st; matching rail/fork/sleeve moves. H-pattern diagram in HUD."),
        Beat("ratios", 20.0,
             "In first, the input gear turns the countershaft more slowly, and a small countershaft gear drives "
             "a large output gear: the engine turns about three and a half times per output-shaft turn. Fourth "
             "locks input to output: one to one. Fifth is an overdrive.",
             "Driving shots (hard cuts between gears): power path glows through the engaged pair. HUD: input "
             "rpm, output rpm and ratio for 1st (3.48:1), then 4th (1:1), then 5th (0.81:1). Glow: headset + selected pair (4th: dogs, sleeve, hub only)."),
        Beat("reverse", 7.5,
             "Reverse adds an idler gear between the shafts, so the output turns backwards.",
             "Cut to reverse: idler highlighted, output shaft turning the opposite way. HUD gear R."),
    ]),
    Scene("s05", "A gear shift, step by step", summary="1st to 2nd at 3000 rpm, slowed 150x.", beats=[
        Beat("intro", 7.0,
             "Here's a shift from first to second at three thousand rpm, slowed right down.",
             "Cutaway gearbox close on the 1-2 synchro and 2nd gear; car driving in 1st at 3000 rpm, clutch "
             "engaged. HUD: engine rpm, 2nd-gear rpm vs output rpm, pedal bar, H-pattern dot, SLOW x150."),
        Beat("clutch_in", 5.5,
             "One: clutch in. The engine is disconnected.",
             "Step card 1. Pedal bar goes down; HUD status DISENGAGED; engine rpm starts falling."),
        Beat("neutral", 5.5,
             "Two: the sleeve slides out of first, into neutral.",
             "Step card 2. Fork moves the 1-2 sleeve off 1st gear's dogs to the centre. Lever dot to N."),
        Beat("sync", 14.0,
             "Three: the blocker ring's cone presses on second gear. Friction slows it, along with the "
             "countershaft, input shaft and clutch disc, until it matches the output shaft's speed.",
             "Step card 3. Sleeve moves toward 2nd, blocker ring touches the cone and indexes to block. HUD "
             "2nd-gear rpm falls to output rpm. Glow on the cone contact."),
        Beat("engage", 8.5,
             "Four: speeds matched, the blocker ring lets the sleeve through, onto second gear's dog teeth.",
             "Step card 4. Sleeve passes the blocker ring and engages 2nd's dog teeth. HUD gear 2."),
        Beat("clutch_out", 9.5,
             "Five: clutch out. The engine is reconnected at about eighteen hundred rpm: same road speed, "
             "taller gear.",
             "Step card 5. Pedal up, brief slip, engine rpm settles at ~1790. HUD ENGAGED."),
    ]),
    Scene("s06", "Final drive and differential", summary="Propshaft, ring and pinion, open diff straight and in a turn.", beats=[
        Beat("prop", 6.5,
             "The propeller shaft carries the drive back to the rear axle.",
             "Camera travels from the gearbox tail along the spinning propshaft (U-joints) to the rear axle."),
        Beat("ringpinion", 14.0,
             "There, a small pinion drives a large ring gear: ten teeth against forty-one, a final drive ratio "
             "of 4.1 to 1, turning the drive through a right angle.",
             "Axle housing cut away. Pinion and ring gear close-up. Labels: Pinion 10T, Ring gear 41T. HUD: "
             "propshaft rpm vs ring rpm."),
        Beat("diffparts", 11.5,
             "The ring gear is bolted to the differential case. Inside, two spider gears mesh with two side "
             "gears, one splined to each driveshaft.",
             "Exploded view of the differential: case, cross-pin, 2 spider gears, 2 side gears, driveshaft "
             "stubs; reassembles. Labels."),
        Beat("straight", 9.0,
             "Going straight, the spider gears don't spin on their pin. Everything turns as one, and both "
             "wheels match.",
             "Straight-line driving: case, spiders, side gears turn as one. HUD: left/right wheel rpm equal."),
        Beat("turn", 18.0,
             "In a turn, the outer wheel travels further than the inner one. The spider gears now spin on their "
             "pin, letting one side speed up as the other slows. The case turns at the average of the two.",
             "Car enters a tight left turn (R 5 m). High view over the rear axle (x-ray). Spiders spin on the "
             "pin. HUD: inner/outer wheel rpm and case rpm = average."),
    ]),
    Scene("s07", "Driveshafts and CV joints", summary="Constant-velocity joints, plunge, wheels, car moves.", beats=[
        Beat("why", 10.5,
             "Each driveshaft has a constant-velocity joint at each end, because the wheel moves up and down "
             "while the differential stays put.",
             "Rear-right driveshaft from behind. Suspension moves the wheel up/down; shaft angle changes. "
             "Labels: Inner joint, Outer joint, Driveshaft."),
        Beat("rzeppa", 16.5,
             "In the outer joint, six balls run in grooves between inner and outer races. A cage holds them in "
             "the plane that splits the angle, so the wheel turns at exactly the shaft's speed.",
             "Cutaway outer (Rzeppa) joint: balls, cage, races; joint articulates. Labels: Balls, Cage, Inner "
             "race, Outer race. HUD: shaft rpm = wheel rpm."),
        Beat("plunge", 6.5,
             "The inner joint can also slide, as the wheel's distance from the differential changes.",
             "Cutaway inner (tripod) joint; rollers slide outboard in their tracks on both bump and droop. Label: Plunge."),
        Beat("moves", 8.0,
             "Finally, the wheel turns, the tire grips the road, and the car moves.",
             "Pull out; body fades back in; slow motion ramps to real time and the car drives off."),
    ]),
    Scene("s08", "Recap", summary="Drive through 1st, 2nd and 3rd with live readouts.", beats=[
        Beat("together", 4.0,
             "Let's put it all together.",
             "Real time. Semi-transparent car stationary in 1st with clutch pressed, engine idling. Tracking "
             "camera alongside. HUD: gear, rpm gauge, km/h, pedal, H-pattern."),
        Beat("first", 6.0,
             "Clutch up in first, and the engine pulls to three thousand rpm.",
             "Clutch slips then locks; car accelerates; rpm climbs to 3000 (24 km/h). Motion blur."),
        Beat("second", 7.0,
             "Clutch in, second gear, clutch out: the revs drop, and the car keeps accelerating.",
             "Quick shift to 2nd: rpm drops to ~1790, then climbs again (40 km/h at 3000)."),
        Beat("third", 5.0,
             "Then third.",
             "Shift to 3rd: rpm drops to ~2010 and climbs."),
        Beat("summary", 10.0,
             "Engine, clutch, gearbox, propeller shaft, final drive, differential, and driveshafts: one chain "
             "of gears and shafts, turning fuel into motion.",
             "Camera pulls back and up as the car cruises; power path glows once more; fade to black."),
    ]),
]


def _layout():
    t = 0.0
    for s in SCENES:
        s.start = t
        bt = 0.0
        for b in s.beats:
            b.start = bt
            b.gstart = t + bt
            bt += b.dur
        t += s.dur


_layout()
SCENE = {s.id: s for s in SCENES}
TOTAL = sum(s.dur for s in SCENES)


def scene(sid):
    return SCENE[sid]


def check(verbose=True):
    """Raise if any narration overruns its beat or a duration isn't frame-exact."""
    problems = []
    for s in SCENES:
        for b in s.beats:
            if abs(b.dur * FPS - round(b.dur * FPS)) > 1e-9:
                problems.append(f"{s.id}.{b.id}: duration {b.dur} not a whole number of frames")
            need = LEAD + b.speak_time + 0.2
            if need > b.dur:
                problems.append(f"{s.id}.{b.id}: needs {need:.2f}s > {b.dur}s ({b.words} words)")
    if verbose:
        words = sum(b.words for s in SCENES for b in s.beats)
        print(f"total {TOTAL:.1f}s ({TOTAL/60:.2f} min), {int(TOTAL*FPS)} frames, {words} words, "
              f"speech {words/WPM:.2f} min")
        for s in SCENES:
            print(f"  {s.id} {s.title:32s} {s.dur:6.1f}s  start {s.start:6.1f}s  frames {s.frames}")
    if problems:
        raise AssertionError("\n".join(problems))
    return True


if __name__ == "__main__":
    check()
