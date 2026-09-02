# TDXDepthCamMerger

A TouchDesigner component that merges the point clouds of several depth cameras into one coordinate
space. It works out how the cameras sit relative to each other using
[Open3D](http://www.open3d.org/), then lines the clouds up.

Version 0.4.0. Built and tested on TouchDesigner **2025.33070**.

Works with:

- Kinect Azure
- Orbbec
- ZED
- any pair of TOPs you supply yourself, one point cloud and one colour. Recorded clouds, other
  sensors and synthetic data all work this way.

Cameras arrive as TOPs, because that is what their TouchDesigner operators give. Inside the
component they become points. What comes out is a POP.

## Coming from 0.0.3

0.4.0 is a breaking change. The install is the part that changed most, so read the next section
carefully.

Version numbers jumped straight from 0.0.3 to 0.4.0. There was no 0.2.0 or 0.3.0 release.

Renamed parameters:

| 0.0.3 | 0.4.0 |
|---|---|
| `Gather kinects` | `Gather devices` |
| `Use player for calibration` | `Use mask for calibration` |
| `Specify pair x` / `Specify pair y` | `Specify pair`, two plain integer fields |
| `Current pair`, `Number of pairs`, `As intermediary` | gone. Use `Reference device` and the calibration table |

New: `Device type`, `Python exe`, `Check worker`, `Voxel size`, `Refine voxel`, `Max range`,
`Use coloured ICP`, `RANSAC seed`, `Preset matrix DAT`, `Rebuild chain`, `Reset calibration`, and
the `Last ...` readouts.

The [walkthrough video](https://vimeo.com/501525725) shows the 0.0.3 interface. The idea is the
same. The names are not.

## Try it without a camera

Open `DemoTDXDepthCamMerger.toe`. It fakes three depth cameras looking at one scene from different
places, coloured red, green and blue, starting misaligned.

Set **Python exe** as described below, pulse **Gather devices**, calibrate pair 1 and 2, then pair
2 and 3. Watch the clouds come together.

## Install

### 1. Drop in the component

Put `TDXDepthCamMerger.tox` into your project.

### 2. Get a Python that has Open3D

This is the step people get stuck on, so here is the whole story.

**Open3D must not go inside TouchDesigner.** The old 0.0.3 instructions told you to copy it into
`TouchDesigner/bin/Lib/site-packages`, or to pip install it into TouchDesigner's own Python. On
current builds that kills TouchDesigner the moment the component loads it. Not an error you can
catch. The whole application disappears. Open3D brings its own copies of libraries TouchDesigner
has already loaded, and the two cannot share a process.

So this component never imports Open3D. It runs the maths in a **separate Python**, outside
TouchDesigner. It writes the clouds to a temp folder, runs that Python as a normal program, and
reads back a 4x4 matrix. TouchDesigner itself only ever uses numpy.

Two good things come out of that. Your TouchDesigner install stays clean, and the component does
not care which Python version TouchDesigner ships.

What you need is **any Python 3.11 or newer with Open3D installed**, as long as it is not
TouchDesigner's own. Install it once and forget it.

With conda, which is the least trouble:

```
conda create -n td python=3.11
conda activate td
pip install open3d numpy
```

With plain Python and a virtual environment:

```
python -m venv td-open3d
td-open3d\Scripts\pip install open3d numpy
```

Tested against Open3D 0.19.0. Nearby versions should be fine.

### 3. Point the component at it, and check

On the **Setup** page:

1. Set **Python exe** to that interpreter's `python.exe`. Not a folder, not `python.bat`, the
   actual executable. Some examples:

   ```
   C:/Users/you/anaconda3/envs/td/python.exe
   C:/Users/you/td-open3d/Scripts/python.exe
   ```

   To find it, activate the environment in a terminal and run:

   ```
   python -c "import sys; print(sys.executable)"
   ```

2. Pulse **Check worker**. This runs that Python once and asks it what it has. It is quick and
   changes nothing.

3. Read the three fields underneath:

   | Field | What you want |
   |---|---|
   | **Worker status** | `OK` |
   | **Open3D version** | a version, for example `0.19.0` |
   | **Python version** | `3.11` or newer, for example `3.11.13` |

**If Worker status does not say `OK`, nothing else in the component will work.** Fix it here
before going on. The field tells you what went wrong:

| Worker status | What it means | Fix |
|---|---|---|
| `FAILED: Python exe is not set...` | The parameter is empty | Set **Python exe** |
| `FAILED: Python exe does not exist: ...` | Wrong path, or the environment moved or was deleted | Check the path. Use the `sys.executable` trick above |
| `FAILED: worker failed: ModuleNotFoundError: No module named 'open3d'` | The Python runs, but has no Open3D. Usually the system Python instead of your environment | `pip install open3d` into that environment, or point at the right `python.exe` |
| `FAILED: worker timed out after 120s` | The Python started but never answered | Try running it in a terminal. Antivirus and network drives are the usual causes |

You only do this once per machine. The component remembers the path in your project.

If you share a project, or move it to another machine, **Python exe** will point somewhere that
does not exist. Set it again and pulse **Check worker**. The shipped `.tox` comes with the field
blank on purpose, so it never carries someone else's path.

## Connect the cameras

The component does not make the camera operators. It reads the ones you make.

1. Put one camera TOP per physical camera in a single base COMP. Name them after the device type
   with the number on the end: `kinectazure1`, `kinectazure2`, or `orbbec1`, `orbbec2`, or `zed1`,
   `zed2`. **The names matter.** Each `Device<n>` looks for the TOP whose number matches.

2. Pin each TOP to one camera: **Sensor** on the Kinect Azure TOP, **Device** on the Orbbec TOP,
   **Camera** on the ZED TOP. Never two TOPs on the same camera.

3. Set **Inputs op** to that base, then pulse **Gather devices**. **Number of devices** should
   match what you plugged in. If it is empty or short, wait and pulse again. An Orbbec can take 30
   seconds to show up after being plugged in.

The component builds the extra select TOPs each device needs. You do not have to.

### Orbbec

Set **Depth Align Mode** on the Orbbec TOP to `Hardware` or `Software`. The colour and depth
cameras sit in slightly different places, so without this the colours land on the wrong points.
The Kinect has the same switch on its select TOP and the component sets that one for you. The
Orbbec select has no such parameter, so this one is yours.

For a Femto Mega or Femto Bolt, use the **Kinect Azure TOP** with **Hardware Type** set to
`Orbbec Compatible`, and set **Device type** here to `Kinect Azure`. You get body tracking that
way, which gives you the player index and therefore masking. Do not mix Orbbec TOPs and Kinect
Azure operators in one project. TouchDesigner warns that this is unstable.

Not yet tested against physical Orbbec hardware. Please open an issue if it misbehaves.

### ZED

Set **Reference Frame** on the ZED TOP to `Camera`, not `World`. In world mode the cloud moves
with the camera's own tracking, which fights the calibration you are measuring.

**Use mask for calibration** is greyed out for ZED. The ZED mask holds body IDs, needs body
tracking running through a ZED CHOP, and the threshold here was tuned for the Kinect player index.
Ask for it if you want it. In the meantime `Custom TOPs` lets you feed any mask you like.

Not yet tested against physical ZED hardware either.

## Calibrate

1. **Setup** page: pick **Device type**, set **Inputs op**, pulse **Gather devices**.

2. **Calibrate** page: set **Specify pair** to two camera numbers. The first is the target and
   does not move. The second is the source and gets moved onto it. **IDs** shows which physical
   cameras those numbers mean.

3. Pulse **Calibrate**.

4. Repeat for each pair, chaining outward: 1 and 2, then 2 and 3, then 3 and 4. Every result is
   stored, and the component composes the chain so everything lands in the frame of
   **Reference device**.

5. Read the merged cloud from `out_pop`.

**Point each camera at its neighbour, not always at camera 1.** Cameras far apart barely see the
same things. Chaining through neighbours is much more accurate.

Other buttons: **Refine** runs ICP again on an existing calibration and never starts a fresh
search. **Reset calibration** puts every device back to identity. **Rebuild chain** recomposes the
transforms without re-registering, which is what you want after changing **Reference device**.

**Refine only reaches a few centimetres.** At the default **Refine voxel** of 0.01 it pulls a
camera back about 0.08 m and no further. If one has been knocked further than that, pulse
**Calibrate** instead. Refine will not rescue it, and it will not tell you it failed to.

### Reading the result

You get **Last status**, **Last fitness**, **Last RMSE**, **Last correspondences** and
**Last overlap**.

**Read Last overlap first.** It is how much of what the target camera sees the source camera also
sees. It decides whether a pair can be calibrated at all:

- **0.55 and above**: 86 to 100% of pairs came out correct in testing, to a few millimetres.
- **Below 0.55**: only 27%. The failures are not small errors, they are metres out.

If it is low, move the cameras so they share more of the scene. No parameter fixes this.

**Last status** says whether the solver settled, not whether the answer is right.

- `OK`: it converged and the numbers look sane.
- `WARN`: marginal. Look at the cloud before trusting it.
- `FAIL`: the result is noise.

**A `WARN` usually means the four runs disagreed.** Every **Calibrate** runs the rough search four
times over and compares the answers. That search is random, so it only disagrees with itself when
the scene is ambiguous. When that happens you get `WARN` even if the fitness looks good.

Treat it as an early warning rather than a complaint about the answer in front of you. It is saying
this pair is unstable, so the next press of **Calibrate** may well land somewhere wrong. More
overlap, or something less symmetric in view, is the fix.

Setting **RANSAC seed** to 0 or more switches this check off. A seeded run comes out identical
every time, so four of them always agree and the agreement tells you nothing.

An `OK` can still be wrong. Two clouds that both contain a floor can be lined up wrongly and still
score well. **Always look at the merged cloud in the viewport.** That is what confirms a
calibration.

Overlap is measured after the fact, so a wrong answer reports the overlap of that wrong answer.
Every pair's numbers are also kept in the `calibrationData` table, so you can see which link in a
chain is the weak one.

## Parameters

### Setup

| Parameter | What it does |
|---|---|
| Device type | `Kinect Azure`, `Orbbec`, `ZED` or `Custom TOPs`. Changing it rebuilds every device's sources |
| Inputs op | The base COMP holding your camera TOPs. Every device type uses it |
| Gather devices | Find the cameras and build a `Device<n>` for each |
| Number of devices, Devices | Read-only. What was found |
| Python exe | The Python that runs the maths. Must have Open3D |
| Check worker | Test that Python and report back |
| Worker status | `OK`, or what went wrong |
| Open3D version, Python version | Read-only. What that Python reported |

### Calibrate

| Parameter | Default | What it does |
|---|---|---|
| Reference device | 1 | Whose frame everything ends up in |
| Specify pair | 1, 2 | Target and source. The target does not move |
| IDs | | Read-only. The physical cameras behind those numbers |
| Calibrate | | Run the current mode on the current pair |
| Refine | | ICP on top of the existing calibration |
| Rebuild chain | | Recompose without re-registering |
| Reset calibration | | Forget everything |
| Last status, fitness, RMSE, correspondences | | Read-only. How the last run went |
| Last overlap | | Read-only. Shared view, 0 to 1. Below 0.55 is unreliable |

### Registration

The defaults are the ones I use.

| Parameter | Default | What it does |
|---|---|---|
| Mode | Global + ICP refine | `Global + ICP refine` does both in one go. `Global registration only` stops after the rough match. `From table DAT` uses a 4x4 you made elsewhere |
| Preset matrix DAT | | A 4x4 table DAT, for `From table DAT` |
| Voxel size (m) | 0.05 | Detail of the rough match. Bigger is faster and coarser |
| Refine voxel (m) | 0.01 | Detail of the refine. 0 means full resolution |
| Max range (m) | 0 | Drop points further away than this. 0 keeps everything. Setting this below your working distance will ruin a calibration |
| Use coloured ICP | off | Use colour as well as shape. Needs colour on both clouds, and only helps when the texture does not repeat |
| Use mask for calibration | off | Calibrate on the masked region only |
| RANSAC seed | -1 | -1 uses every core, and the same scene can give slightly different results each run. Set 0 or more for a repeatable answer, at the cost of running on one thread |

**Use mask for calibration** helps when the cameras see a lot of static background. It needs a mask
to exist. Kinect Azure has one, the player index. Orbbec and ZED do not, so the toggle is greyed
out. With `Custom TOPs` it turns on as soon as one row supplies a mask.

## Output

One output, `out_pop`. A POP holding every camera's cloud merged into one point set, in the
reference camera's frame, with two attributes:

- `P`, the position
- `Color`, the colour that camera saw there

Only real points are in it. Pixels the sensor reported as invalid are dropped inside each device,
before the merge, so the count is honest and every colour sits on its own point.

Drop it into a geometry COMP, feed it to instancing, or keep going with POPs.

**Coming from 0.2.0:** this replaces the `out_points` and `out_colors` TOP outputs. Use a
`POP to TOP` if you need the old shape.

## Custom TOP sources

Set **Device type** to `Custom TOPs` and fill in the `customSources` table inside the component,
one row per camera:

| name | pointcloud | color | mask |
|---|---|---|---|
| cam1 | `/project1/clouds/cloud1` | `/project1/clouds/color1` | |
| cam2 | `/project1/clouds/cloud2` | `/project1/clouds/color2` | |

Then pulse **Gather devices** and calibrate as usual. The colour TOP does not have to match the
point cloud's resolution, it is sampled per point. Leave `mask` empty if you have none.

This is also how you add a camera the component does not know yet. If your sensor has a TOP that
outputs a point cloud, add a row to the `deviceTypes` table. No code needed.

## Limits

Cameras need real overlap, seen from broadly similar angles. More than about 90 degrees apart, or
facing each other, does not give the matcher enough in common.

Get around it by chaining: calibrate 1 to 2 and 2 to 3, and camera 3 lands in camera 1's frame even
though 1 and 3 never saw the same thing.

A large flat floor with little else cannot be calibrated reliably. A floor looks the same after
turning or sliding it, so a wrong answer fits just as well as the right one. Give the cameras
something with shape to look at.

This is a one-off calibration step, not something that runs every frame. Expect seconds.

## Development

| Path | What |
|---|---|
| `src/` | the Python files that become DATs in the component |
| `tests/` | suites that need no TouchDesigner |
| `tools/` | build and test scripts that run inside TouchDesigner |

`src/worker.py` runs outside TouchDesigner and works on its own:

```
python src/worker.py --probe
python src/worker.py job.json
```

The job format is at the top of that file. The matrix it returns satisfies `x_target = M @ x_source`
with column vectors.

Tests:

```
python tests/test_pipeline.py                    # numpy only
python tests/test_worker.py path/to/python.exe   # drives the worker as a subprocess
```

`test_pipeline.py` checks that the TouchDesigner side never imports Open3D, since that is the thing
that crashes the process.

## Support

Follow me on [Instagram](https://www.instagram.com/darien.brito/) or
[Twitter](https://twitter.com/DarienBrito).

To support the work: [Patreon](https://www.patreon.com/c/darienbrito)

Problems and requests: [issue tracker](https://github.com/DarienBrito/TDXDepthCamMerger/issues)

Best,
Darien

Darien Brito, 2026. MIT licensed, see [LICENSE.md](LICENSE.md).
