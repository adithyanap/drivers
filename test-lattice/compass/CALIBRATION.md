# SE100 / IST8310 heading calibration

## What the firmware does at boot

After the IST8310 identity check, the application enters a 500-sample
calibration run.  It prints:

```
CALIBRATION: rotate the SE100 slowly through a full 360 degrees.
Keep it level, away from metal/magnets. Starting in 3 s;
keep rotating until all five dots have printed.
```

## How to collect the measurements

1. Put the module on a non-magnetic surface.  Keep it level.
2. Reset/power the board and wait for the calibration message.
3. After the 3-second delay, rotate the **whole module** slowly through a
   full 360 degrees in the horizontal plane.  One continuous rotation is
   enough; do not tilt it more than necessary.
4. Keep rotating until the five progress dots have printed (roughly 12-20
   seconds, depending on measurement time).
5. The firmware prints the result, for example:

   ```
   Calibration OK: hard-iron X=221 Y=129 counts; soft-iron X=0.751 Y=1.498 (samples=500)
   ```

   The numbers above are only an example format.  The firmware does **not**
   contain any hard-coded calibration values; your values come from your
   rotation.

If the run is rejected, the output says so:

- `code -2`: fewer than 80 samples were collected.
- `code -3`: one axis did not move enough, or the module was not rotated
  through a useful part of the circle.  Repeat the rotation more slowly and
  through a larger angle.

On rejection the firmware resets the correction to identity (no correction)
and continues printing raw X/Y.  It never invents calibration values.

## What is corrected

- **Hard iron**: the centre of the X/Y circle, subtracted from the raw counts.
- **Soft iron**: the X/Y radius mismatch, normalised with a Q12 scale factor.

The correction is applied to raw X/Y before converting to microtesla and
before calculating heading.  Z is printed but is not used by the level-heading
calculation, so it is not part of the 2D heading calibration.

## Heading configuration

In `compass/main.c`:

```c
#define APP_DECLINATION_TENTHS    0   /* e.g. -12.3 deg -> -123 */
#define APP_HEADING_OFFSET_TENTHS 0   /* mounting/axis offset */
```

Heading is `atan2(-Y, X)`, in tenths of a degree:

- `0.0` = north
- `90.0` = east
- `180.0` = south
- `270.0` = west

The heading path is integer-only.  It does not use the toolchain's soft-float
`atan2f()`, which produced incorrect quadrants on this RV32I build.
