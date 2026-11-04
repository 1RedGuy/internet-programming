# Reaction Arcade enclosure (Fusion 360 script)

`ReactionArcade.py` builds the complete 3D-printable enclosure for the Reaction Arcade kit
(reaction-time game for 10-14 year olds) in a **new** Fusion 360 design:

| Component | Colour | What it is | Print orientation |
|---|---|---|---|
| `BASE` | dark grey #2B2D31 | floor + 4 walls cut at the 15° slope, 4 tilted corner pillars with M3 heat-set inserts, ESP32 cradle (pads, rails, snap lips, end stop), buzzer cradle + 5-slot grille, USB cutout with 1 mm chamfered frame, 2 cable-tie loops, 4 foot recesses, seam reveal | floor on the bed |
| `TOP_PANEL` | off-white #EDEDE8 | 3 mm sloped plate with alignment lip, display window/pocket/2 snap clips, stepped diffuser seat + LED-ring holder (stop tabs, 2 clips, crush ribs), 4 × 16.2 mm button holes, 4 counterbored M3 holes, engraved "REACTION" | visible face **down** on the bed |
| `DIFFUSER` | translucent white | 1.2 mm press-fit disc, Ø ring_od + 6 | visible face down |
| `FIT_TEST` | off-white | coupon: button hole, diffuser seat, insert pillar stub, USB cutout in a wall | plate on the bed |
| `Reference parts` | light grey | `REF_ESP32`, `REF_USB_PLUG`, `REF_BUZZER`, `REF_DISPLAY`, `REF_LED_RING`, `REF_BUTTON_1..4` (do not print) | – |

## Run it

1. Copy the `ReactionArcade` folder (it holds `ReactionArcade.py` and `ReactionArcade.manifest`) to your computer.
2. In Fusion: **Utilities > Add-Ins > Scripts and Add-Ins**, then click **+** next to *My Scripts*. On newer versions
   choose *Script or add-in from device*. Pick the folder.
3. Select **ReactionArcade** and click **Run**. The build takes about a minute. A message box then shows the summary;
   the full report goes to **Text Commands** (*View > Show Text Commands*).

## Parameters

Every dimension is a User Parameter (**Modify > Change Parameters**). All sketches, planes and features are computed
from them when the script runs. These finishing values are also linked live: `top_fillet`, `bed_chamfer`,
`usb_chamfer`, `foot_depth`, `title_depth`, `clip_w`.

To change anything else, edit the parameter, keep that design open and active, and run the script again. It reads
the current parameter values (expressions included) and builds a fresh, consistent design in a new document.

Most likely to tune:

| Parameter | Default | Why |
|---|---|---|
| `disp_y` | 38 mm | With the spec defaults the two rear buttons overlap the display pocket. The validation suggests **45 mm** (or `btn_ang` 39.5°). |
| `btn_r` | 45 mm | Spreads or tightens the 4 buttons around the ring (checked against display, ring, pads, lip, floor). |
| `fit` | 0.3 mm | Slip-fit clearance for the lid lip, display pocket and ESP32 rails. Tune it with FIT_TEST. |
| `press` | 0.1 mm | Diffuser seat interference. Tune it with FIT_TEST. |
| `insert_d` / `insert_depth` | 4.0 / 5.5 mm | Match your heat-set inserts. Tune them with the FIT_TEST pillar stub. |

## After printing

- **TOP_PANEL:** push an M3 screw (or a 3 mm drill) once through each counterbore. Each hole has a 0.25 mm
  sacrificial bridge layer so the face-down print needs no supports.
- **Heat-set inserts:** press them in along the pillar axis, which is perpendicular to the panel (tilted 15°).
- **LED ring:** press it into the tube from inside until it sits on the two stop tabs, with an LED gap facing each tab.
  The clips and crush ribs hold it 4 mm behind the diffuser.
- **Buzzer:** slide it in from the front until its face touches the back wall behind the grille.
- **ESP32:** push the board down between the rails, USB end toward the back wall, until the snap lips click over the
  PCB edge.
- **Display:** push it into the pocket from inside until both clips catch the backpack PCB.
- **Hardware:**
  - 4 × M3 × 8 button-head screws (ISO 7380)
  - 4 × M3 heat-set inserts of 5 mm or less (Ø4.0 hole)
  - 4 × Ø10 mm rubber feet
  - 2 cable ties, 4 mm wide or less

## What the validation reports

- **Overall size and slope:** the computed slope and the one measured on the panel face.
- **Top perimeter:** which round was built (bed-safe round or a fallback).
- **Button-to-floor clearance:** per button; a warning below 5 mm.
- **Panel layout conflicts:** webs under 2 mm, nut keep-outs against display, ring, pads and lip, with a suggested fix.
- **Snap-fit strains:** for the ESP32 lips, display clips and ring clips.
- **Interference:** every REF part against every printed part, REF against REF, and printed against printed.
- **Summary:** every part is BUILT or FAILED, and every skipped optional step is listed. A failed step's partial
  features are rolled back.
